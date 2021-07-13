# C# AutoToString()

```c#
    public static class ObjectExtension
    {
        public static String AutoToString(this Object o)
        {
            StringBuilder sb = new StringBuilder();

            foreach (System.Reflection.PropertyInfo property in o.GetType().GetProperties())
            {
                sb.Append(property.Name);
                sb.Append(": ");
                if (property.GetIndexParameters().Length > 0)
                {
                    sb.Append("Indexed Property cannot be used");
                }
                else
                {
                    sb.Append(property.GetValue(o, null));
                }

                sb.Append(System.Environment.NewLine);
            }
            return sb.ToString();
        }
    }

```

