using Unity.Entities;
using Unity.Jobs;
using Unity.Mathematics;
using Unity.Burst;
using Unity.Collections;
using UnityEngine.Experimental.AI;
using System.Collections.Generic;
using Unity.Transforms;

public class Unit_System : SystemBase
{
    private NavMeshQuery query;
    private float3 extents;
    private Dictionary<int, float3[]> allPaths;
    private List<Entity> routedEntities;
    private List<NativeArray<int>> statusOutputs;
    private List<NativeArray<float3>> results;
    private List<NavMeshQuery> queries;
    private NavMeshWorld navMeshWorld;
    private List<JobHandle> jobHandles;

    protected override void OnCreate()
    {
        extents = new float3(10, 10, 10);
        allPaths = new Dictionary<int, float3[]>();
        statusOutputs = new List<NativeArray<int>>();
        results = new List<NativeArray<float3>>();
        routedEntities = new List<Entity>();
        queries = new List<NavMeshQuery>();
        jobHandles = new List<JobHandle>();

        for(int n=0; n <= 32; n++)
        {
            NativeArray<float3> result = new NativeArray<float3>(1024, Allocator.Persistent);
            NativeArray<int> statusOutput = new NativeArray<int>(3, Allocator.Persistent);
            statusOutputs.Add(statusOutput);
            results.Add(result);
        }
        navMeshWorld = NavMeshWorld.GetDefaultWorld();
    }

    protected override void OnUpdate()
    {
        float deltaTime = Time.DeltaTime;
        int i = 0;
        Entities.
            WithNone<Unit_Routed>().
            WithoutBurst().
            WithStructuralChanges().
            ForEach((Entity e, ref Unit_Component uc, ref DynamicBuffer<Unit_Buffer> ub) =>
        {
            if (i <= UnitManager.instance.maxEntitiesRoutedPerFrame)
            {
                int fromKey = ((int)uc.fromLocation.x + (int)uc.fromLocation.y + (int)uc.fromLocation.z) * UnitManager.instance.maxPathSize;
                int toKey = ((int)uc.toLocation.x + (int)uc.toLocation.y + (int)uc.toLocation.z) * UnitManager.instance.maxPathSize;
                int key = fromKey + toKey;
                //Cached path
                if (UnitManager.instance.useCache && allPaths.ContainsKey(key) && !uc.routed)
                {
                    allPaths.TryGetValue(key, out float3[] cachedPath);
                    for (int h=0; h<cachedPath.Length;h++)
                    {
                        ub.Add(new Unit_Buffer { wayPoints = cachedPath[h] });
                    }
                    uc.routed = true;
                    uc.usingCachedPath = true;
                    EntityManager.AddComponent<Unit_Routed>(e);
                    return;
                }
                //Job
                else if (!uc.routed)
                {
                    NavMeshQuery currentQuery = new NavMeshQuery(navMeshWorld, Allocator.Persistent, UnitManager.instance.maxPathNodePoolSize);
                    SinglePathFindingJob spfj = new SinglePathFindingJob()
                    {
                        query = currentQuery,
                        nml_FromLocation = uc.nml_FromLocation,
                        nml_ToLocation = uc.nml_ToLocation,
                        fromLocation = uc.fromLocation,
                        toLocation = uc.toLocation,
                        extents = extents,
                        maxIteration = UnitManager.instance.maxIterations,
                        result = results[i],
                        statusOutput = statusOutputs[i],
                        maxPathSize = UnitManager.instance.maxPathSize,
                        ub = ub
                    };
                    routedEntities.Add(e);
                    queries.Add(currentQuery);
                    jobHandles.Add(spfj.Schedule());
                }
                i++;
            }
            else
            {
                return;
            }
        }).Run();

        int n = 0;
        NativeArray<JobHandle> jhs = new NativeArray<JobHandle>(jobHandles.Count, Allocator.Temp);
        foreach (JobHandle jh in jobHandles )
        {
            jhs[n] = jh;
            n++;
        }
        JobHandle.CompleteAll(jhs);
        jhs.Dispose();

        int j = 0;
        foreach (JobHandle jh in jobHandles)
        {
            if (statusOutputs[j][0] == 1)
            {
                if (UnitManager.instance.useCache && !allPaths.ContainsKey(statusOutputs[j][1]))
                {
                    float3[] wayPoints = new float3[statusOutputs[j][2]];
                    for (int k=0;k<statusOutputs[j][2];k++)
                    {
                        wayPoints[k] = results[j][k];
                    }
                    if (wayPoints.Length > 0)
                    {
                        allPaths.Add(statusOutputs[j][1], wayPoints);
                    }
                }
                Unit_Component uc = EntityManager.GetComponentData<Unit_Component>(routedEntities[j]);
                uc.routed = true;
                EntityManager.SetComponentData<Unit_Component>(routedEntities[j], uc);
                EntityManager.AddComponent<Unit_Routed>(routedEntities[j]);
            }
            queries[j].Dispose();
            j++;
        }
        routedEntities.Clear();
        jobHandles.Clear();
        queries.Clear();

        //Movement
        Entities
            .WithAll<Unit_Routed>().ForEach((ref Unit_Component uc, ref DynamicBuffer<Unit_Buffer> ub, ref Translation trans) =>
        {
            if (ub.Length>0 && uc.routed)
            {
                uc.waypointDirection = math.normalize(ub[uc.currentBufferIndex].wayPoints - trans.Value);
                trans.Value += uc.waypointDirection * uc.speed * deltaTime;
                if (!uc.reached && math.distance(trans.Value, ub[uc.currentBufferIndex].wayPoints) <= uc.minDistanceReached && uc.currentBufferIndex < ub.Length-1)
                {
                    uc.currentBufferIndex = uc.currentBufferIndex + 1;
                    if (uc.currentBufferIndex == ub.Length-1)
                    {
                        uc.reached = true;
                    }
                }
                else if (uc.reached && math.distance(trans.Value, ub[uc.currentBufferIndex].wayPoints) <= uc.minDistanceReached && uc.currentBufferIndex > 0)
                {
                    uc.currentBufferIndex = uc.currentBufferIndex - 1;
                    if (uc.currentBufferIndex == 0)
                    {
                        uc.reached = false;
                    }
                }
            }
        }).ScheduleParallel();
    }

    protected override void OnDestroy()
    {
        for (int n=0; n<=UnitManager.instance.maxEntitiesRoutedPerFrame; n++)
        {
            statusOutputs[n].Dispose();
            results[n].Dispose();
        }
    }

    [BurstCompile]
    private struct SinglePathFindingJob : IJob
    {
        PathQueryStatus status;
        PathQueryStatus returningStatus;
        public NavMeshQuery query;
        public NavMeshLocation nml_FromLocation;
        public NavMeshLocation nml_ToLocation;
        public float3 fromLocation;
        public float3 toLocation;
        public float3 extents;
        public int maxIteration;
        public DynamicBuffer<Unit_Buffer> ub;
        public NativeArray<float3> result;
        public NativeArray<int> statusOutput;
        public int maxPathSize;

        public void Execute()
        {
            nml_FromLocation = query.MapLocation(fromLocation, extents, 0);
            nml_ToLocation = query.MapLocation(toLocation, extents, 0);
            if (query.IsValid(nml_FromLocation) && query.IsValid(nml_ToLocation))
            {
                status = query.BeginFindPath(nml_FromLocation, nml_ToLocation, -1);
                if (status == PathQueryStatus.InProgress)
                {
                    status = query.UpdateFindPath(maxIteration, out int iterationPerformed);
                }
                if (status == PathQueryStatus.Success)
                {
                    status = query.EndFindPath(out int polygonSize);
                    NativeArray<NavMeshLocation> res = new NativeArray<NavMeshLocation>(polygonSize, Allocator.Temp);
                    NativeArray<StraightPathFlags> straightPathFlag = new NativeArray<StraightPathFlags>(maxPathSize, Allocator.Temp);
                    NativeArray<float> vertexSide = new NativeArray<float>(maxPathSize, Allocator.Temp);
                    NativeArray<PolygonId> polys = new NativeArray<PolygonId>(polygonSize, Allocator.Temp);
                    int straightPathCount = 0;
                    query.GetPathResult(polys);
                    returningStatus = PathUtils.FindStraightPath(
                        query,
                        fromLocation,
                        toLocation,
                        polys,
                        polygonSize,
                        ref res,
                        ref straightPathFlag,
                        ref vertexSide,
                        ref straightPathCount,
                        maxPathSize
                        );
                    if(returningStatus == PathQueryStatus.Success)
                    {
                        int fromKey = ((int)fromLocation.x + (int)fromLocation.y + (int)fromLocation.z) * maxPathSize;
                        int toKey = ((int)toLocation.x + (int)toLocation.y + (int)toLocation.z) * maxPathSize;
                        int key = fromKey + toKey;
                        statusOutput[0] = 1;
                        statusOutput[1] = key;
                        statusOutput[2] = straightPathCount;
                        
                        for (int i=0; i<straightPathCount; i++)
                        {
                            result[i] = (float3)res[i].position + new float3(0, 0.75f, 0); // elevated point
                            ub.Add(new Unit_Buffer { wayPoints = result[i] });
                        }
                    }
                    res.Dispose();
                    straightPathFlag.Dispose();
                    polys.Dispose();
                    vertexSide.Dispose();
                }
            }
        }
    }
}

public struct Unit_Routed : IComponentData { }
