/**
 * @file test_notebook_compat.cpp
 * @brief Test notebook integration features (silent mode, clf)
 */

#include <iostream>
#include <fstream>
#include <vector>
#include <cstdio>
#include <cppplot/cppplot.hpp>
#include <cppplot/control/control.hpp>

using namespace std;
using namespace cppplot;

int main() {
    int passed = 0, failed = 0;
    
    cout << "====== Notebook Compatibility Tests ======\n\n";
    
    // Test 1: set_silent and is_silent
    cout << "TEST: set_silent/is_silent ... ";
    {
        set_silent(true);
        if (is_silent()) {
            set_silent(false);
            if (!is_silent()) {
                cout << "PASSED\n";
                passed++;
            } else {
                cout << "FAILED (set_silent(false) didn't work)\n";
                failed++;
            }
        } else {
            cout << "FAILED (set_silent(true) didn't work)\n";
            failed++;
        }
    }
    
    // Test 2: savefig respects silent mode
    cout << "TEST: savefig respects silent mode ... ";
    {
        set_silent(true);
        
        auto x = linspace(0, 10, 50);
        std::vector<double> y;
        for (auto xi : x) y.push_back(sin(xi));
        plot(x, y);
        savefig("should_not_exist_test.svg");
        
        // Check if file was NOT created
        std::ifstream f("should_not_exist_test.svg");
        if (!f.good()) {
            cout << "PASSED\n";
            passed++;
        } else {
            cout << "FAILED (file was created in silent mode)\n";
            failed++;
            f.close();
            std::remove("should_not_exist_test.svg");
        }
        
        set_silent(false);
    }
    
    // Test 3: clf() works
    cout << "TEST: clf() clears figure ... ";
    {
        clf();  // Should not throw
        cout << "PASSED\n";
        passed++;
    }
    
    // Test 4: control module accessible
    cout << "TEST: control module accessible ... ";
    {
        using namespace cppplot::control;
        TransferFunction G({1}, {1, 2, 1});
        if (G.order() == 2) {
            cout << "PASSED\n";
            passed++;
        } else {
            cout << "FAILED\n";
            failed++;
        }
    }
    
    // Test 5: Matrix class accessible
    cout << "TEST: Matrix class accessible ... ";
    {
        Matrix A = {{1, 2}, {3, 4}};
        if (A.rows == 2 && A.cols == 2) {
            cout << "PASSED\n";
            passed++;
        } else {
            cout << "FAILED\n";
            failed++;
        }
    }
    
    cout << "\n====== Results: " << passed << " passed, " << failed << " failed ======\n";
    return failed;
}
