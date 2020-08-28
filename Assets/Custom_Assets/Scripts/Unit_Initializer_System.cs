using Unity.Entities;
using Unity.Transforms;
using Unity.Burst;
using Unity.Mathematics;

public class Unit_Initializer_System : SystemBase
{
    BeginInitializationEntityCommandBufferSystem bi_ECB;
    public float elapsedTime;

    protected override void OnCreate()
    {
        bi_ECB = World.GetOrCreateSystem<BeginInitializationEntityCommandBufferSystem>();
        elapsedTime = 0;
    }
    protected override void OnUpdate()
    {
        var ecb = bi_ECB.CreateCommandBuffer().AsParallelWriter();
        elapsedTime += Time.DeltaTime;
        if (elapsedTime > UnitManager.instance.spawnEvery)
        {
            elapsedTime = 0;
            Entities
                .WithBurst(synchronousCompilation: true)
                .ForEach((Entity e, int entityInQueryIndex, in Unit_Initializer_Component uic, in LocalToWorld ltw) =>
            {
                for (int i = 0; i < uic.xGridCount; i++)
                {
                    for (int j = 0; j < uic.zGridCount; j++)
                    {
                        Entity defEntity = ecb.Instantiate(entityInQueryIndex, uic.prefabToSpawn);
                        float3 position = new float3(i * uic.xPadding, uic.baseOffset, j * uic.zPadding) + uic.currentPosition;
                        ecb.SetComponent(entityInQueryIndex, defEntity, new Translation { Value = position });
                        ecb.AddComponent<Unit_Component>(entityInQueryIndex, defEntity);
                        ecb.AddBuffer<Unit_Buffer>(entityInQueryIndex, defEntity);

                        Unit_Component uc = new Unit_Component();
                        uc.fromLocation = position;
                        uc.toLocation = new float3(position.x, position.y, position.z + uic.destinationDistanceZAxis);
                        uc.currentBufferIndex = 0;
                        uc.speed = (float)new Random(uic.seed+(uint)(i*j)).NextDouble(uic.minSpeed, uic.maxSpeed);
                        uc.minDistanceReached = uic.minDistanceReached;
                        ecb.SetComponent(entityInQueryIndex, defEntity, uc);
                    }
                }
                //ecb.DestroyEntity(entityInQueryIndex, e);
            }).ScheduleParallel();
        }
        bi_ECB.AddJobHandleForProducer(Dependency);
    }
}
