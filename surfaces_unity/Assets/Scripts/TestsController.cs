using System;
using System.Collections.Generic;
using Library.Generic;
using UnityEngine;
using UnityEngine.UI;

public class TestsController : MonoBehaviour
{
    // Start is called before the first frame update
    private void Awake() {
        var exportPathDataButton = GameObject.Find("RunTestsButton").GetComponent<Button>();
        exportPathDataButton.onClick.AddListener(RunTests);
    }

    private void RunTests() {
        TestMatrixDot();
        TestMatrixLU();
        TestInverseMatrix();
        TestTransposeMatrix();
        TestComplex();
        TestLessSquareEquation();
        Debug.Log("Tests complete");
    }

    private void TestMatrixDot() {
        var m1 = new Matrix(new List<List<double>> {
            new List<double>{2.0, 1.0, 3.0},
            new List<double>{5.0, 3.0, 0.0},
            new List<double>{9.0, 7.0, 5.0},
        });
        Debug.Assert(Math.Abs(m1.GetDet() - 29.0) < double.Epsilon);

        var m2 = new Matrix(new List<List<double>> {
            new List<double>{2.0, 1.0, 0.0},
            new List<double>{9.0, 3.0, 0.0},
            new List<double>{9.0, 9.0, 0.0},
        });
        Debug.Assert(Math.Abs(m2.GetDet()) < double.Epsilon);

        Debug.Log("Test matrix dot complete");
    }

    private void TestMatrixLU() {
        var m = new Matrix(new List<List<double>> {
            new List<double>{2.0, 1.0, 3.0},
            new List<double>{5.0, 3.0, 0.0},
            new List<double>{9.0, 7.0, 5.0},
        });

        var l = new Matrix(new List<List<double>> {
            new List<double>{1.0, 0.0, 0.0},
            new List<double>{2.5, 1.0, 0.0},
            new List<double>{4.5, 5.0, 1.0},
        });
        var u = new Matrix(new List<List<double>> {
            new List<double>{2.0, 1.0, 3.0},
            new List<double>{0.0, 0.5, -7.5},
            new List<double>{0.0, 0.0, 29},
        });

        var lu = m.GetLU();
        var l1 = lu.Key;
        var u1 = lu.Value;

        Debug.Assert(l1 == l);
        Debug.Assert(u1 == u);

        Debug.Log("Test matrix lu complete");
    }

    private void TestInverseMatrix() {
        var m = new Matrix(new List<List<double>> {
            new List<double>{2.0, 1.0, 3.0},
            new List<double>{5.0, 3.0, 0.0},
            new List<double>{9.0, 7.0, 5.0},
        });
        var m1 = m.Inverse();
        Debug.Assert(Math.Abs((m * m1).GetDet() - 1) < 1e-8);

        Debug.Log("Test inverse matrix complete");
    }

    private void TestTransposeMatrix() {
        var m = new Matrix(new List<List<double>> {
            new List<double>{2.0, 1.0, 3.0},
            new List<double>{5.0, 3.0, 0.0},
            new List<double>{9.0, 7.0, 5.0},
        });
        var mt = m.Transpose();

        var m0 = new Matrix(new List<List<double>> {
            new List<double>{2.0, 5.0, 9.0},
            new List<double>{1.0, 3.0, 7.0},
            new List<double>{3.0, 0.0, 5.0},
        });

        Debug.Assert(m0 == mt);

        Debug.Log("Test transpose matrix complete");
    }

    private void TestComplex() {
        var m = new Matrix(new List<List<double>> {
            new List<double>{2.0, 1.0, 3.0, 4.5},
            new List<double>{5.0, 3.0, 0.0, 4.0},
            new List<double>{9.0, 7.0, 5.0, 7.5},
        });
        Debug.Assert(m.RowCount == 3);
        Debug.Assert(m.ColumnCount == 4);

        var m1 = m.Transpose();
        Debug.Assert(m1.RowCount == 4);
        Debug.Assert(m1.ColumnCount == 3);

        var m2 = m * m1;
        Debug.Assert(m2.RowCount == 3);
        Debug.Assert(m2.ColumnCount == 3);

        var m3 = m1 * m;
        Debug.Assert(m3.RowCount == 4);
        Debug.Assert(m3.ColumnCount == 4);
    }

    private void TestLessSquareEquation() {
        // var a = new Matrix(new List<List<double>> {
        //     new List<double>{1},
        //     new List<double>{1},
        //     new List<double>{1},
        // });
        // var b = new Matrix(new List<List<double>> {
        //     new List<double>{1},
        //     new List<double>{3},
        //     new List<double>{5},
        // });
        var a = new Matrix(new List<List<double>> {
            new List<double>{1, 2},
            new List<double>{1, 3},
            new List<double>{1, 5},
        });
        var b = new Matrix(new List<List<double>> {
            new List<double>{4},
            new List<double>{5},
            new List<double>{3},
        });

        var result = Equations.LessEquations(a, b);
        var q = a * result - b;
    }
}
