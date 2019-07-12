http://cassandra.apache.org/doc/latest/cql/operators.html

# Arithmetic Operators

CQL supports the following operators:

| Operator  | Description                         |
| --------- | ----------------------------------- |
| - (unary) | Negates operand                     |
| +         | Addition                            |
| -         | Substraction                        |
| *         | Multiplication                      |
| /         | Division                            |
| %         | Returns the remainder of a division |

## Number Arithmetic

All arithmetic operations are supported on numeric types or counters.

The return type of the operation will be based on the operand types:

| left/right   | tinyint  | smallint | int     | bigint  | counter | float   | double  | varint  | decimal |
| ------------ | -------- | -------- | ------- | ------- | ------- | ------- | ------- | ------- | ------- |
| **tinyint**  | tinyint  | smallint | int     | bigint  | bigint  | float   | double  | varint  | decimal |
| **smallint** | smallint | smallint | int     | bigint  | bigint  | float   | double  | varint  | decimal |
| **int**      | int      | int      | int     | bigint  | bigint  | float   | double  | varint  | decimal |
| **bigint**   | bigint   | bigint   | bigint  | bigint  | bigint  | double  | double  | varint  | decimal |
| **counter**  | bigint   | bigint   | bigint  | bigint  | bigint  | double  | double  | varint  | decimal |
| **float**    | float    | float    | float   | double  | double  | float   | double  | decimal | decimal |
| **double**   | double   | double   | double  | double  | double  | double  | double  | decimal | decimal |
| **varint**   | varint   | varint   | varint  | decimal | decimal | decimal | decimal | decimal | decimal |
| **decimal**  | decimal  | decimal  | decimal | decimal | decimal | decimal | decimal | decimal | decimal |

`*`, `/` and `%` operators have a higher precedence level than `+` and `-` operator. By consequence, they will be evaluated before. If two operator in an expression have the same precedence level, they will be evaluated left to right based on their position in the expression.

## Datetime Arithmetic

A `duration` can be added (+) or substracted (-) from a `timestamp` or a `date` to create a new `timestamp` or `date`. So for instance:

```
SELECT * FROM myTable WHERE t = '2017-01-01' - 2d
```

will select all the records with a value of `t` which is in the last 2 days of 2016.