using Unity.Entities;
using Unity.Mathematics;

[GenerateAuthoringComponent]
public struct Unit_Initializer_Component : IComponentData
{
    public int xGridCount;
    public int zGridCount;
    public float baseOffset;
    public float xPadding;
    public float zPadding;
    public Entity prefabToSpawn;
    //New
    public float3 currentPosition;
    public int destinationDistanceZAxis;
    public int minSpeed;
    public int maxSpeed;
    public float minDistanceReached;
    public uint seed;
}
