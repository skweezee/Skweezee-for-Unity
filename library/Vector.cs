using System;


public class Vector
{
    private float[] vector;

    public Vector(float[] vector)
    {
        
        this.vector = vector;

    }

    public float[] Array()
    {

        return vector;

    }

    public float Component(int i)
    {

        return vector[i];

    }

    public int Dimension()
    {

        return vector.Length;

    }

    public float Magnitude()
    {

        float mag = 0;

        for (int i = 0; i < Dimension(); i++)
        {

            mag += vector[i] * vector[i];

        }

        return (float)Math.Sqrt(mag);

    }

    static public float Magnitude(float[] vector)
    {

        float mag = 0;

        for (int i = 0; i < vector.Length; i++)
        {

            mag += vector[i] * vector[i];

        }

        return (float)Math.Sqrt(mag);

    }

    public float[] Direction()
    {

        float[] dir = new float[Dimension()];

        float mag = Magnitude();

        for (int i = 0; i < Dimension(); i++)
        {

            dir[i] = vector[i] / mag;

        }

        return dir;

    }

    public float[] Direction(float[] vector)
    {

        float[] dir = new float[vector.Length];

        float mag = new Vector(vector).Magnitude();

        for (int i = 0; i < vector.Length; i++)
        {

            dir[i] = vector[i] / mag;

        }

        return dir;

    }

    public float Dot(Vector other)
    {

        float dot = 0;

        if (other.Dimension() == this.Dimension())
        {

            float[] u = this.Direction();
            float[] v = other.Direction();

            for (int i = 0; i < Dimension(); i++)
            {

                dot += u[i] * v[i];

            }

        }

        return dot;

    }

    public float Dot(float[] other)
    {

        float dot = 0;

        if (other.Length == this.Dimension())
        {

            float[] u = this.Direction();
            float[] v = new Vector(other).Direction();

            for (int i = 0; i < u.Length; i++)
            {

                dot += u[i] * v[i];

            }

        }

        return dot;

    }

    static public float Dot(float[] one, float[] other)
    {

        float dot = 0;

        if (one.Length == other.Length)
        {

            float[] u = new Vector(one).Direction();
            float[] v = new Vector(other).Direction();

            for (int i = 0; i < u.Length; i++)
            {

                dot += u[i] * v[i];

            }

        }

        return dot;

    }

    public static float[] Square(float[] vec)
    {
        int dimensions = vec.Length;
        float[] square = new float[dimensions];
        for (int i = 0; i < dimensions; i++)
        {
            square[i] = vec[i] * vec[i];
        }
        return square;
    }

}

