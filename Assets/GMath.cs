
using UnityEngine;

public static class GMath 
{
    public static Vector2 ComplexMUL(Vector2 pos, Vector2 dir)
    {
        return new Vector2(pos.x * dir.x - pos.y * dir.y, pos.x * dir.y + pos.y * dir.x);
    }

    public static Vector2 ComplexConjugateMUL(Vector2 pos, Vector2 dir)
    {
        return new Vector2(pos.x * dir.x + pos.y * dir.y, pos.y * dir.x - pos.x * dir.y);
    }
}
