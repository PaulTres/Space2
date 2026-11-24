using System;
using System.Collections.Generic;
using System.Globalization;
using System.Text;

namespace Nav3D.Demo
{
    //Reads a .json into dictionaries, lists and strings, keyed access without a mirror class — JsonUtility only fills typed fields.
    //Objects, arrays and strings only; anything else raises a FormatException the caller reports.
    public static class DemoJson
    {
        #region Public methods

        public static Dictionary<string, object> Parse(string _Text)
        {
            int position = 0;

            object root = ReadValue(_Text, ref position);

            SkipWhitespace(_Text, ref position);

            if (position != _Text.Length)
                throw new FormatException($"unexpected text after the root object at character {position}");

            return AsObject(root);
        }

        //accessors that name the missing key instead of throwing KeyNotFoundException at the caller
        public static Dictionary<string, object> Object(object _Value)
        {
            return AsObject(_Value);
        }

        public static string String(Dictionary<string, object> _Node, string _Key)
        {
            object value = Member(_Node, _Key);

            if (value is not string text)
                throw new FormatException($"\"{_Key}\" is not a string");

            return text;
        }

        public static List<object> Array(Dictionary<string, object> _Node, string _Key)
        {
            object value = Member(_Node, _Key);

            if (value is not List<object> items)
                throw new FormatException($"\"{_Key}\" is not an array");

            if (items.Count == 0)
                throw new FormatException($"\"{_Key}\" is empty");

            return items;
        }

        public static string[] Strings(Dictionary<string, object> _Node, string _Key)
        {
            List<object> items  = Array(_Node, _Key);
            string[]     result = new string[items.Count];

            for (int i = 0; i < items.Count; i++)
            {
                if (items[i] is not string text)
                    throw new FormatException($"\"{_Key}\" holds something other than a string");

                result[i] = text;
            }

            return result;
        }

        #endregion

        #region Service methods

        static object Member(Dictionary<string, object> _Node, string _Key)
        {
            if (!_Node.TryGetValue(_Key, out object value))
                throw new FormatException($"no \"{_Key}\"");

            return value;
        }

        static Dictionary<string, object> AsObject(object _Value)
        {
            return _Value is not Dictionary<string, object> node ? throw new FormatException("expected an object") : node;

        }

        static object ReadValue(string _Text, ref int _Position)
        {
            SkipWhitespace(_Text, ref _Position);

            if (_Position >= _Text.Length)
                throw new FormatException("the file ends where a value was expected");

            return _Text[_Position] switch
            {
                '{' => ReadObject(_Text, ref _Position),
                '[' => ReadArray(_Text, ref _Position),
                '"' => ReadString(_Text, ref _Position),
                _ => throw new FormatException($"only objects, arrays and strings are supported (character {_Position})")
            };

        }

        static Dictionary<string, object> ReadObject(string _Text, ref int _Position)
        {
            Dictionary<string, object> node = new Dictionary<string, object>();

            _Position++;

            SkipWhitespace(_Text, ref _Position);

            if (Take(_Text, ref _Position, '}'))
                return node;

            while (true)
            {
                SkipWhitespace(_Text, ref _Position);

                string key = ReadString(_Text, ref _Position);

                SkipWhitespace(_Text, ref _Position);

                if (!Take(_Text, ref _Position, ':'))
                    throw new FormatException($"expected ':' after \"{key}\" at character {_Position}");

                node[key] = ReadValue(_Text, ref _Position);

                SkipWhitespace(_Text, ref _Position);

                if (Take(_Text, ref _Position, ',')) continue;
                if (Take(_Text, ref _Position, '}')) return node;

                throw new FormatException($"expected ',' or '}}' after \"{key}\" at character {_Position}");
            }
        }

        static List<object> ReadArray(string _Text, ref int _Position)
        {
            List<object> items = new List<object>();

            _Position++;

            SkipWhitespace(_Text, ref _Position);

            if (Take(_Text, ref _Position, ']'))
                return items;

            while (true)
            {
                items.Add(ReadValue(_Text, ref _Position));

                SkipWhitespace(_Text, ref _Position);

                if (Take(_Text, ref _Position, ',')) continue;
                if (Take(_Text, ref _Position, ']')) return items;

                throw new FormatException($"expected ',' or ']' at character {_Position}");
            }
        }

        static string ReadString(string _Text, ref int _Position)
        {
            if (!Take(_Text, ref _Position, '"'))
                throw new FormatException($"expected a string at character {_Position}");

            StringBuilder builder = new StringBuilder();

            while (true)
            {
                if (_Position >= _Text.Length)
                    throw new FormatException("the file ends inside a string");

                char symbol = _Text[_Position++];

                if (symbol == '"')
                    return builder.ToString();

                if (symbol != '\\')
                {
                    builder.Append(symbol);

                    continue;
                }

                char escaped = _Text[_Position++];

                switch (escaped)
                {
                    case '"':  builder.Append('"');  break;
                    case '\\': builder.Append('\\'); break;
                    case '/':  builder.Append('/');  break;
                    case 'b':  builder.Append('\b'); break;
                    case 'f':  builder.Append('\f'); break;
                    case 'n':  builder.Append('\n'); break;
                    case 'r':  builder.Append('\r'); break;
                    case 't':  builder.Append('\t'); break;
                    case 'u':
                        builder.Append((char) int.Parse(_Text.Substring(_Position, 4), NumberStyles.HexNumber));

                        _Position += 4;

                        break;
                    default: throw new FormatException($"unknown escape \\{escaped} at character {_Position}");
                }
            }
        }

        static bool Take(string _Text, ref int _Position, char _Symbol)
        {
            if (_Position >= _Text.Length || _Text[_Position] != _Symbol)
                return false;

            _Position++;

            return true;
        }

        static void SkipWhitespace(string _Text, ref int _Position)
        {
            while (_Position < _Text.Length && char.IsWhiteSpace(_Text[_Position]))
                _Position++;
        }

        #endregion
    }
}
