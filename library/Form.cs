using System.Collections.Generic;


public class Form
    {

        List<float[]> templates;
        private string label;

        public Form(float[] template)
        {
            templates = new List<float[]>();
            templates.Add(template);
        }

        public Form()
        {

            templates = new List<float[]>();
            label = "";

        }

        public Form(string label)
        {

            templates = new List<float[]>();
            this.label = label;

        }

        public Form(string label, float[] template)
        {

            templates = new List<float[]>();
            templates.Add(template);

            this.label = label;

        }
    

        public void Record(float[] template)
        {

            templates = new List<float[]>();
            templates.Add(template);

        }
        
        public void Record()
        {


            templates = new List<float[]>();
            templates.Add(Skweezee.GetVector());

        }
        

        public void Synonym(float[] template)
        {

            templates.Add(template);

        }

        public string GetLabel()
        {

            return label;

        }
        

        public bool Is(string label)
        {

            return this.label.Equals(label);

        }
        

        
        public float Fit()
        {

            float fit = 0;

            for (int i = 0; i < templates.Count; i++)
            {

                float dot = Vector.Dot(templates[i], Skweezee.GetVector());

                if (dot > fit)
                {

                    fit = dot;

                }


            }

            return fit;

        }
        
        
        public float Fit(float[] v)
        {

            float fit = 0;

            for (int i = 0; i < templates.Count; i++)
            {

                float dot = Vector.Dot(templates[i], v);

                if (dot > fit)
                {

                    fit = dot;

                }


            }

            return fit;

        }
        
    }