from manim import *
import numpy as np

class MatrixOperations(Scene):
    def construct(self):

        # Create the different lines of text
        param1 = MathTex(r"\text{Parameters:}").align_to(LEFT)
        param2 = MathTex(r"\text{a     : Link length (distance along x-axis)}").align_to(LEFT)
        param3 = MathTex(r"\alpha : \text{Link twist (angle around x-axis) in radians}").align_to(LEFT)
        param4 = MathTex(r"\text{d     : Link offset (distance along z-axis)}").align_to(LEFT)
        param5 = MathTex(r"\theta : \text{Joint angle (rotation around z-axis) in radians}").align_to(LEFT)

        # Create a VGroup to group the lines together
        params = VGroup(param1, param2, param3, param4, param5).arrange(DOWN, buff=0.5)

        # Show the parameters
        self.play(Write(params))
        self.wait(1)

        # Fade out everything
        self.play(FadeOut(params))

        # Define matrices A and B using NumPy for automatic computation
        A = np.array([[1, 2], [3, 4]])
        B = np.array([[2, 0], [1, 2]])
        
        # Compute matrix operations using NumPy
        result_add = Matrix(A + B)
        result_sub = Matrix(A - B)
        result_mult = Matrix(np.dot(A,B))  # @ operator is matrix multiplication
        
        # Convert A and B to Mantolistim Matrix objects
        matrix_A = Matrix(A.tolist())
        matrix_B = Matrix(B.tolist())
        
        # Arrange operations in a structured format
        matrices_group = VGroup(
            MathTex("A_{T}^{O}"), matrix_A, MathTex("+"), MathTex("B"), matrix_B, MathTex("= A + B"), result_add
        ).arrange(RIGHT, buff=0.5)
        
        self.play(Write(matrices_group))
        self.wait(1)
        
        # Show matrix subtraction
        matrices_group_sub = VGroup(
            MathTex("A"), matrix_A, MathTex("-"), MathTex("B"), matrix_B, MathTex("= A - B"), result_sub
        ).arrange(RIGHT, buff=0.5)
        
        self.play(Transform(matrices_group, matrices_group_sub))
        self.wait(1)
        
        # Show matrix multiplication
        matrices_group_mult = VGroup(
            MathTex("A"), matrix_A, MathTex("\cdot"), MathTex("B"), matrix_B, MathTex("= A \cdot B"), result_mult
        ).arrange(RIGHT, buff=0.5)
        
        self.play(Transform(matrices_group, matrices_group_mult))
        self.wait(2)
        
        # Fade out everything
        self.play(FadeOut(matrices_group))
