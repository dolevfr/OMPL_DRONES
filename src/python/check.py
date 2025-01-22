import sympy as sp

# Example SymPy vector
vector = sp.Matrix([1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20])

# Extract the 4th to 7th elements (0-based indexing: 3 to 6 inclusive)
sliced_vector1 = vector[3:7]
sliced_vector2 = vector[7:11]
sliced_vector3 = vector[13:17]


print(sliced_vector1)
print(sliced_vector2)
print(sliced_vector3)
