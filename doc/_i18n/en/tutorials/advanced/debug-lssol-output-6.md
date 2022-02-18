This page will help you to debug the issue known as `lssol output (6)` that you might typically encounter as the following:

```
lssol output (6):
An input parameter is invalid

QP failed to run()
```

Note: while this apply specifically to LSSOL, this might be helpful to debug other QP solver failures.

As the message says, one of the input to the solver is invalid. LSSOL returns this when one of the following occurs (from most likely to unlikely):

1. The bounds on a constraint \\(b_l \leq A_i^T x \leq b_u \\) or a variable \\(b_l \leq x_i \leq b_u\\) are invalid, i.e.
  - \\(b_u < b_l\\) (most likely)
  - \\(b_l = b\_u\\) and \\(\mid b_l \mid > bigbnd\\)
2. The warm start data are invalid
3. A version of the solver is called that uses a permutation \\(k_x\\) and \\(k_x\\) is invalid
4. Not enough workspace memory was allocated

The file `/tmp/fort.9` should inform you on the exact nature of the problem. You should have a line ressembling one of these:

- `XXX  The bounds on  varbl  3  are inconsistent.   bl =   -1000.000       bu =   -10000.00` (inconsistent bounds on a variable)
- `XXX  The bounds on  lncon  4  are inconsistent.   bl =    2       bu =   -1.000000` (inconsistent bounds on an inequality constraint)
- `XXX  The equal bounds on  lncon  1  are infinite.   Bounds =   0.1000000+101  bigbnd =   0.9999900E+10` (\\(b\\) too large in an equality constraint \\(A_i^T x = b\\) - represented internally as \\(b \leq A_i^T x \leq b\\))

Note: on Windows, that file is located in the folder where the application using LSSOL is executed
