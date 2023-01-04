# pip install numpy
# start
# https://numpy.org/doc/stable/user/quickstart.html

import numpy as np

# ein Vektor als Liste
a = [1, 2, 3]
# eine Matrix als verschachtelte Liste
b = [[1, 2, 3], [4, 5, 6], [7, 8, 9]]

# automatischer Datentyp
an = np.array(a)
# manuell gesetzter Datentyp
bn = np.array(b, dtype=np.float64)

print(a)
print(b)
print(an)
print(bn)

# Datentypen
print(type(b))
print(type(bn))
print(bn.dtype)

# Dimensionen der Matrix
print(bn.shape)

# Zugriff auf ein Element bn[Zeile, Spalte]
print(bn[1, 2])

# transponieren
bn = bn.T
print(bn)
print(bn.dtype)

# elementweise Multiplikation
cn = bn * bn
print(f"elementweise Multiplikation \n {cn}")

# Matrix-Multiplikation
# dn = bn.dot(bn)
dn = bn @ bn
print(f"Matrix-Multiplikation \n {dn}")

# Matrix-Vektor Multiplikation
en = bn @ an
print(en)

# Teilmatrix
# alles ab zeile 1 spalte 1 zb.
# [[1. 4. 7.]
#  [2. 5. 8.]
#  [3. 6. 9.]]
# ->
# [[5. 8.]
#  [6. 9.]]
print(bn[1:, 1:])

# von 0 bis 3 in 2er Schritten
# [[1. 4. 7.]
#  [2. 5. 8.]
#  [3. 6. 9.]]
# ->
# [[1. 7.]
#  [3. 9.]]
print(bn[0:3:2, 0:3:2])

# Zeilen vertauschen
# [erstes, nulltes, erstes] -element
print(f"from \n {bn} \n to \n {bn[[1, 0, 1], :]}")

# Einheitsmatrix
print(np.eye(3))
