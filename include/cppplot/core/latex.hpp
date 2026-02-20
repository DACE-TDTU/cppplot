/**
 * @file latex.hpp
 * @brief LaTeX to SVG/Unicode converter for mathematical expressions
 * 
 * Supports:
 * - Greek letters: \alpha, \beta, \gamma, etc.
 * - Superscript: x^2, x^{10}
 * - Subscript: x_1, x_{12}
 * - Math operators: \sum, \int, \prod, \sqrt, etc.
 * - Fractions: \frac{a}{b} (rendered as a/b or using SVG)
 * - Special symbols: \infty, \pm, \times, \div, etc.
 */

#ifndef CPPPLOT_CORE_LATEX_HPP
#define CPPPLOT_CORE_LATEX_HPP

#include <string>
#include <map>
#include <vector>
#include <sstream>
#include <regex>

namespace cppplot {

/**
 * @brief LaTeX to Unicode/SVG converter
 */
class LaTeXRenderer {
public:
    /**
     * @brief Render mode
     */
    enum class RenderMode {
        Unicode,    // Convert to Unicode characters (simple)
        SVG,        // Generate SVG elements (complex)
        MathML      // Generate MathML (for foreignObject)
    };
    
private:
    // Greek lowercase letters
    static const std::map<std::string, std::string>& greekLower() {
        static const std::map<std::string, std::string> m = {
            {"alpha", "α"}, {"beta", "β"}, {"gamma", "γ"}, {"delta", "δ"},
            {"epsilon", "ε"}, {"zeta", "ζ"}, {"eta", "η"}, {"theta", "θ"},
            {"iota", "ι"}, {"kappa", "κ"}, {"lambda", "λ"}, {"mu", "μ"},
            {"nu", "ν"}, {"xi", "ξ"}, {"omicron", "ο"}, {"pi", "π"},
            {"rho", "ρ"}, {"sigma", "σ"}, {"tau", "τ"}, {"upsilon", "υ"},
            {"phi", "φ"}, {"chi", "χ"}, {"psi", "ψ"}, {"omega", "ω"},
            {"varepsilon", "ε"}, {"vartheta", "ϑ"}, {"varpi", "ϖ"},
            {"varrho", "ϱ"}, {"varsigma", "ς"}, {"varphi", "ϕ"}
        };
        return m;
    }
    
    // Greek uppercase letters
    static const std::map<std::string, std::string>& greekUpper() {
        static const std::map<std::string, std::string> m = {
            {"Alpha", "Α"}, {"Beta", "Β"}, {"Gamma", "Γ"}, {"Delta", "Δ"},
            {"Epsilon", "Ε"}, {"Zeta", "Ζ"}, {"Eta", "Η"}, {"Theta", "Θ"},
            {"Iota", "Ι"}, {"Kappa", "Κ"}, {"Lambda", "Λ"}, {"Mu", "Μ"},
            {"Nu", "Ν"}, {"Xi", "Ξ"}, {"Omicron", "Ο"}, {"Pi", "Π"},
            {"Rho", "Ρ"}, {"Sigma", "Σ"}, {"Tau", "Τ"}, {"Upsilon", "Υ"},
            {"Phi", "Φ"}, {"Chi", "Χ"}, {"Psi", "Ψ"}, {"Omega", "Ω"}
        };
        return m;
    }
    
    // Math operators and symbols
    static const std::map<std::string, std::string>& mathSymbols() {
        static const std::map<std::string, std::string> m = {
            // Operators
            {"sum", "∑"}, {"prod", "∏"}, {"int", "∫"}, {"iint", "∬"},
            {"iiint", "∭"}, {"oint", "∮"}, {"sqrt", "√"},
            {"partial", "∂"}, {"nabla", "∇"}, {"infty", "∞"},
            
            // Relations
            {"leq", "≤"}, {"geq", "≥"}, {"neq", "≠"}, {"approx", "≈"},
            {"equiv", "≡"}, {"sim", "∼"}, {"propto", "∝"},
            {"ll", "≪"}, {"gg", "≫"}, {"subset", "⊂"}, {"supset", "⊃"},
            {"subseteq", "⊆"}, {"supseteq", "⊇"}, {"in", "∈"}, {"ni", "∋"},
            {"notin", "∉"},
            
            // Arrows
            {"to", "→"}, {"rightarrow", "→"}, {"leftarrow", "←"},
            {"Rightarrow", "⇒"}, {"Leftarrow", "⇐"}, {"leftrightarrow", "↔"},
            {"Leftrightarrow", "⇔"}, {"uparrow", "↑"}, {"downarrow", "↓"},
            {"mapsto", "↦"},
            
            // Binary operators
            {"pm", "±"}, {"mp", "∓"}, {"times", "×"}, {"div", "÷"},
            {"cdot", "·"}, {"ast", "∗"}, {"star", "⋆"}, {"circ", "∘"},
            {"bullet", "•"}, {"oplus", "⊕"}, {"otimes", "⊗"},
            {"cap", "∩"}, {"cup", "∪"}, {"wedge", "∧"}, {"vee", "∨"},
            
            // Misc symbols
            {"forall", "∀"}, {"exists", "∃"}, {"nexists", "∄"},
            {"neg", "¬"}, {"angle", "∠"}, {"triangle", "△"},
            {"prime", "′"}, {"emptyset", "∅"}, {"aleph", "ℵ"},
            {"hbar", "ℏ"}, {"ell", "ℓ"}, {"Re", "ℜ"}, {"Im", "ℑ"},
            
            // Dots
            {"ldots", "…"}, {"cdots", "⋯"}, {"vdots", "⋮"}, {"ddots", "⋱"},
            
            // Brackets (for reference)
            {"langle", "⟨"}, {"rangle", "⟩"}, {"lceil", "⌈"}, {"rceil", "⌉"},
            {"lfloor", "⌊"}, {"rfloor", "⌋"}, {"lbrace", "{"}, {"rbrace", "}"},
            
            // Special
            {"degree", "°"}, {"celsius", "℃"}, {"permil", "‰"},
            {"dagger", "†"}, {"ddagger", "‡"}, {"S", "§"}, {"P", "¶"}
        };
        return m;
    }
    
    // Superscript digits and letters
    static const std::map<char, std::string>& superscripts() {
        static const std::map<char, std::string> m = {
            {'0', "⁰"}, {'1', "¹"}, {'2', "²"}, {'3', "³"}, {'4', "⁴"},
            {'5', "⁵"}, {'6', "⁶"}, {'7', "⁷"}, {'8', "⁸"}, {'9', "⁹"},
            {'+', "⁺"}, {'-', "⁻"}, {'=', "⁼"}, {'(', "⁽"}, {')', "⁾"},
            {'n', "ⁿ"}, {'i', "ⁱ"}, {'x', "ˣ"}, {'y', "ʸ"}, {'a', "ᵃ"},
            {'b', "ᵇ"}, {'c', "ᶜ"}, {'d', "ᵈ"}, {'e', "ᵉ"}, {'f', "ᶠ"},
            {'g', "ᵍ"}, {'h', "ʰ"}, {'j', "ʲ"}, {'k', "ᵏ"}, {'l', "ˡ"},
            {'m', "ᵐ"}, {'o', "ᵒ"}, {'p', "ᵖ"}, {'r', "ʳ"}, {'s', "ˢ"},
            {'t', "ᵗ"}, {'u', "ᵘ"}, {'v', "ᵛ"}, {'w', "ʷ"}, {'z', "ᶻ"}
        };
        return m;
    }
    
    // Subscript digits and letters
    static const std::map<char, std::string>& subscripts() {
        static const std::map<char, std::string> m = {
            {'0', "₀"}, {'1', "₁"}, {'2', "₂"}, {'3', "₃"}, {'4', "₄"},
            {'5', "₅"}, {'6', "₆"}, {'7', "₇"}, {'8', "₈"}, {'9', "₉"},
            {'+', "₊"}, {'-', "₋"}, {'=', "₌"}, {'(', "₍"}, {')', "₎"},
            {'a', "ₐ"}, {'e', "ₑ"}, {'h', "ₕ"}, {'i', "ᵢ"}, {'j', "ⱼ"},
            {'k', "ₖ"}, {'l', "ₗ"}, {'m', "ₘ"}, {'n', "ₙ"}, {'o', "ₒ"},
            {'p', "ₚ"}, {'r', "ᵣ"}, {'s', "ₛ"}, {'t', "ₜ"}, {'u', "ᵤ"},
            {'v', "ᵥ"}, {'x', "ₓ"}
        };
        return m;
    }
    
    // Math font styles
    static const std::map<std::string, std::pair<std::string, std::string>>& mathFonts() {
        static const std::map<std::string, std::pair<std::string, std::string>> m = {
            // {command, {start_offset_upper, start_offset_lower}}
            {"mathbb", {"𝔸", "𝕒"}},   // Blackboard bold (double-struck)
            {"mathcal", {"𝒜", "𝒶"}},  // Calligraphic
            {"mathfrak", {"𝔄", "𝔞"}}, // Fraktur
            {"mathbf", {"𝐀", "𝐚"}},   // Bold
            {"mathit", {"𝐴", "𝑎"}},   // Italic
            {"mathrm", {"A", "a"}}     // Roman (normal)
        };
        return m;
    }

public:
    /**
     * @brief Convert LaTeX string to Unicode
     * @param latex Input LaTeX string (with or without $ delimiters)
     * @return Unicode string
     */
    static std::string toUnicode(const std::string& latex) {
        std::string input = latex;
        
        // Remove $ delimiters if present
        if (!input.empty() && input.front() == '$') {
            input = input.substr(1);
        }
        if (!input.empty() && input.back() == '$') {
            input = input.substr(0, input.length() - 1);
        }
        
        std::string result;
        size_t i = 0;
        
        while (i < input.length()) {
            // Check for backslash commands
            if (input[i] == '\\') {
                size_t cmdStart = i + 1;
                size_t cmdEnd = cmdStart;
                
                // Find command name (letters only)
                while (cmdEnd < input.length() && std::isalpha(input[cmdEnd])) {
                    cmdEnd++;
                }
                
                if (cmdEnd > cmdStart) {
                    std::string cmd = input.substr(cmdStart, cmdEnd - cmdStart);
                    
                    // Check Greek lowercase
                    auto& gl = greekLower();
                    if (gl.count(cmd)) {
                        result += gl.at(cmd);
                        i = cmdEnd;
                        continue;
                    }
                    
                    // Check Greek uppercase
                    auto& gu = greekUpper();
                    if (gu.count(cmd)) {
                        result += gu.at(cmd);
                        i = cmdEnd;
                        continue;
                    }
                    
                    // Check math symbols
                    auto& ms = mathSymbols();
                    if (ms.count(cmd)) {
                        result += ms.at(cmd);
                        i = cmdEnd;
                        continue;
                    }
                    
                    // Handle \frac{a}{b}
                    if (cmd == "frac") {
                        i = cmdEnd;
                        std::string num = extractBraced(input, i);
                        std::string den = extractBraced(input, i);
                        result += "(" + toUnicode(num) + "/" + toUnicode(den) + ")";
                        continue;
                    }
                    
                    // Handle \sqrt{x} or \sqrt[n]{x}
                    if (cmd == "sqrt") {
                        i = cmdEnd;
                        // Check for optional [n]
                        std::string index;
                        if (i < input.length() && input[i] == '[') {
                            i++;
                            while (i < input.length() && input[i] != ']') {
                                index += input[i++];
                            }
                            if (i < input.length()) i++; // skip ]
                        }
                        std::string content = extractBraced(input, i);
                        if (index.empty()) {
                            result += "√(" + toUnicode(content) + ")";
                        } else {
                            result += toSuperscript(index) + "√(" + toUnicode(content) + ")";
                        }
                        continue;
                    }
                    
                    // Handle \text{...}
                    if (cmd == "text" || cmd == "mathrm" || cmd == "textrm") {
                        i = cmdEnd;
                        std::string content = extractBraced(input, i);
                        result += content;
                        continue;
                    }
                    
                    // Handle space commands
                    if (cmd == "quad") { result += "  "; i = cmdEnd; continue; }
                    if (cmd == "qquad") { result += "    "; i = cmdEnd; continue; }
                    if (cmd == "," || cmd == ";" || cmd == "!") { result += " "; i = cmdEnd; continue; }
                    
                    // Unknown command - keep as is
                    result += "\\" + cmd;
                    i = cmdEnd;
                    continue;
                }
                
                // Handle \\ (line break - just add newline for now)
                if (cmdStart < input.length() && input[cmdStart] == '\\') {
                    result += "\n";
                    i = cmdStart + 1;
                    continue;
                }
                
                // Single special character after backslash
                if (cmdStart < input.length()) {
                    char c = input[cmdStart];
                    if (c == ' ' || c == ',' || c == ';' || c == '!' || c == ':') {
                        result += " ";
                        i = cmdStart + 1;
                        continue;
                    }
                    if (c == '{' || c == '}' || c == '\\' || c == '%' || c == '$' || c == '&' || c == '#' || c == '_') {
                        result += c;
                        i = cmdStart + 1;
                        continue;
                    }
                }
            }
            
            // Handle superscript ^
            if (input[i] == '^') {
                i++;
                std::string sup;
                if (i < input.length() && input[i] == '{') {
                    sup = extractBraced(input, i);
                } else if (i < input.length()) {
                    sup = input[i++];
                }
                // First convert any LaTeX commands inside the superscript
                std::string parsedSup = toUnicode(sup);
                result += toSuperscript(parsedSup);
                continue;
            }
            
            // Handle subscript _
            if (input[i] == '_') {
                i++;
                std::string sub;
                if (i < input.length() && input[i] == '{') {
                    sub = extractBraced(input, i);
                } else if (i < input.length()) {
                    sub = input[i++];
                }
                // First convert any LaTeX commands inside the subscript
                std::string parsedSub = toUnicode(sub);
                result += toSubscript(parsedSub);
                continue;
            }
            
            // Handle braces
            if (input[i] == '{') {
                std::string content = extractBraced(input, i);
                result += toUnicode(content);
                continue;
            }
            
            // Skip whitespace in math mode (add thin space)
            if (input[i] == ' ' || input[i] == '\t' || input[i] == '\n') {
                i++;
                continue;
            }
            
            // Regular character
            result += input[i++];
        }
        
        return result;
    }
    
    /**
     * @brief Unicode superscript characters for Greek and special symbols
     */
    static const std::map<std::string, std::string>& unicodeSuperscripts() {
        static const std::map<std::string, std::string> m = {
            // Greek letters (no standard Unicode superscripts, use normal)
            {"α", "ᵅ"}, {"β", "ᵝ"}, {"γ", "ᵞ"}, {"δ", "ᵟ"},
            {"ε", "ᵋ"}, {"θ", "ᶿ"}, {"ι", "ᶥ"}, {"φ", "ᵠ"}, {"χ", "ᵡ"},
            {"π", "π"}, {"μ", "μ"}, {"ν", "ν"}, {"ρ", "ρ"},  // Keep as-is (no super version)
            // Common math
            {"∞", "∞"}, {"+", "⁺"}, {"-", "⁻"}, {"=", "⁼"},
            {"(", "⁽"}, {")", "⁾"}, {"/", "ᐟ"}
        };
        return m;
    }
    
    /**
     * @brief Unicode subscript characters for Greek and special symbols
     */
    static const std::map<std::string, std::string>& unicodeSubscripts() {
        static const std::map<std::string, std::string> m = {
            // Greek letters (limited availability)
            {"β", "ᵦ"}, {"γ", "ᵧ"}, {"ρ", "ᵨ"}, {"φ", "ᵩ"}, {"χ", "ᵪ"},
            {"π", "π"}, {"α", "α"}, {"μ", "μ"},  // Keep as-is
            // Common math
            {"+", "₊"}, {"-", "₋"}, {"=", "₌"},
            {"(", "₍"}, {")", "₎"}
        };
        return m;
    }
    
    /**
     * @brief Convert to superscript Unicode
     */
    static std::string toSuperscript(const std::string& text) {
        std::string result;
        auto& sup = superscripts();
        auto& usup = unicodeSuperscripts();
        
        // Process text character by character (handle UTF-8)
        size_t i = 0;
        while (i < text.length()) {
            // Check if it's a multi-byte UTF-8 character
            unsigned char c = text[i];
            size_t charLen = 1;
            if ((c & 0xE0) == 0xC0) charLen = 2;
            else if ((c & 0xF0) == 0xE0) charLen = 3;
            else if ((c & 0xF8) == 0xF0) charLen = 4;
            
            if (charLen > 1 && i + charLen <= text.length()) {
                // Multi-byte character
                std::string mb = text.substr(i, charLen);
                if (usup.count(mb)) {
                    result += usup.at(mb);
                } else {
                    result += mb;  // Keep as-is
                }
                i += charLen;
            } else {
                // Single byte ASCII
                char ch = text[i];
                if (sup.count(ch)) {
                    result += sup.at(ch);
                } else {
                    result += ch;
                }
                i++;
            }
        }
        return result;
    }
    
    /**
     * @brief Convert to subscript Unicode
     */
    static std::string toSubscript(const std::string& text) {
        std::string result;
        auto& sub = subscripts();
        auto& usub = unicodeSubscripts();
        
        // Process text character by character (handle UTF-8)
        size_t i = 0;
        while (i < text.length()) {
            // Check if it's a multi-byte UTF-8 character
            unsigned char c = text[i];
            size_t charLen = 1;
            if ((c & 0xE0) == 0xC0) charLen = 2;
            else if ((c & 0xF0) == 0xE0) charLen = 3;
            else if ((c & 0xF8) == 0xF0) charLen = 4;
            
            if (charLen > 1 && i + charLen <= text.length()) {
                // Multi-byte character
                std::string mb = text.substr(i, charLen);
                if (usub.count(mb)) {
                    result += usub.at(mb);
                } else {
                    result += mb;  // Keep as-is
                }
                i += charLen;
            } else {
                // Single byte ASCII
                char ch = text[i];
                if (sub.count(ch)) {
                    result += sub.at(ch);
                } else {
                    result += ch;
                }
                i++;
            }
        }
        return result;
    }
    
    /**
     * @brief Generate MathML for SVG foreignObject
     * @param latex LaTeX expression
     * @return MathML string
     */
    static std::string toMathML(const std::string& latex) {
        std::string input = latex;
        
        // Remove $ delimiters
        if (!input.empty() && input.front() == '$') {
            input = input.substr(1);
        }
        if (!input.empty() && input.back() == '$') {
            input = input.substr(0, input.length() - 1);
        }
        
        std::ostringstream ss;
        ss << "<math xmlns=\"http://www.w3.org/1998/Math/MathML\">";
        ss << parseMathML(input);
        ss << "</math>";
        return ss.str();
    }
    
    /**
     * @brief Check if string contains LaTeX commands
     */
    static bool hasLaTeX(const std::string& text) {
        // Check for common LaTeX indicators
        return text.find('\\') != std::string::npos ||
               text.find('^') != std::string::npos ||
               text.find('_') != std::string::npos ||
               (text.find('$') != std::string::npos);
    }
    
    /**
     * @brief Render LaTeX as SVG text element with tspans
     * @param latex LaTeX expression
     * @param x X coordinate
     * @param y Y coordinate
     * @param fontSize Font size
     * @param color Text color
     * @return SVG text element string
     */
    static std::string toSVGText(const std::string& latex, double x, double y,
                                  double fontSize, const std::string& color,
                                  const std::string& anchor = "start",
                                  const std::string& baseline = "alphabetic") {
        std::string unicode = toUnicode(latex);
        std::ostringstream ss;
        
        ss << "<text x=\"" << x << "\" y=\"" << y << "\" "
           << "font-family=\"STIX Two Math, Cambria Math, Latin Modern Math, serif\" "
           << "font-size=\"" << fontSize << "\" "
           << "fill=\"" << color << "\" "
           << "text-anchor=\"" << anchor << "\" "
           << "dominant-baseline=\"" << baseline << "\">"
           << escapeXML(unicode)
           << "</text>\n";
        
        return ss.str();
    }
    
    /**
     * @brief Render LaTeX using SVG foreignObject with MathML
     * @param latex LaTeX expression
     * @param x X coordinate  
     * @param y Y coordinate
     * @param fontSize Font size
     * @param color Text color
     * @return SVG foreignObject element string
     */
    static std::string toSVGForeignObject(const std::string& latex, double x, double y,
                                           double fontSize, const std::string& color,
                                           double width = 200, double height = 50) {
        std::ostringstream ss;
        
        ss << "<foreignObject x=\"" << x << "\" y=\"" << (y - height/2) << "\" "
           << "width=\"" << width << "\" height=\"" << height << "\">\n"
           << "  <div xmlns=\"http://www.w3.org/1999/xhtml\" "
           << "style=\"font-size:" << fontSize << "px; color:" << color << "; "
           << "font-family: 'STIX Two Math', 'Cambria Math', serif;\">\n"
           << "    " << toMathML(latex) << "\n"
           << "  </div>\n"
           << "</foreignObject>\n";
        
        return ss.str();
    }
    
private:
    /**
     * @brief Extract content from braces {...}
     */
    static std::string extractBraced(const std::string& input, size_t& pos) {
        if (pos >= input.length() || input[pos] != '{') {
            return "";
        }
        
        pos++; // skip opening {
        std::string result;
        int depth = 1;
        
        while (pos < input.length() && depth > 0) {
            if (input[pos] == '{') depth++;
            else if (input[pos] == '}') depth--;
            
            if (depth > 0) {
                result += input[pos];
            }
            pos++;
        }
        
        return result;
    }
    
    /**
     * @brief Parse LaTeX to MathML (simplified)
     */
    static std::string parseMathML(const std::string& input) {
        std::ostringstream ss;
        size_t i = 0;
        
        while (i < input.length()) {
            // Handle backslash commands
            if (input[i] == '\\') {
                size_t cmdStart = i + 1;
                size_t cmdEnd = cmdStart;
                while (cmdEnd < input.length() && std::isalpha(input[cmdEnd])) {
                    cmdEnd++;
                }
                
                if (cmdEnd > cmdStart) {
                    std::string cmd = input.substr(cmdStart, cmdEnd - cmdStart);
                    
                    // Greek letters
                    auto& gl = greekLower();
                    auto& gu = greekUpper();
                    if (gl.count(cmd)) {
                        ss << "<mi>" << gl.at(cmd) << "</mi>";
                        i = cmdEnd;
                        continue;
                    }
                    if (gu.count(cmd)) {
                        ss << "<mi>" << gu.at(cmd) << "</mi>";
                        i = cmdEnd;
                        continue;
                    }
                    
                    // Math symbols as operators
                    auto& ms = mathSymbols();
                    if (ms.count(cmd)) {
                        ss << "<mo>" << ms.at(cmd) << "</mo>";
                        i = cmdEnd;
                        continue;
                    }
                    
                    // Fractions
                    if (cmd == "frac") {
                        i = cmdEnd;
                        std::string num = extractBraced(input, i);
                        std::string den = extractBraced(input, i);
                        ss << "<mfrac><mrow>" << parseMathML(num) << "</mrow>"
                           << "<mrow>" << parseMathML(den) << "</mrow></mfrac>";
                        continue;
                    }
                    
                    // Square root
                    if (cmd == "sqrt") {
                        i = cmdEnd;
                        std::string index;
                        if (i < input.length() && input[i] == '[') {
                            i++;
                            while (i < input.length() && input[i] != ']') {
                                index += input[i++];
                            }
                            if (i < input.length()) i++;
                        }
                        std::string content = extractBraced(input, i);
                        if (index.empty()) {
                            ss << "<msqrt>" << parseMathML(content) << "</msqrt>";
                        } else {
                            ss << "<mroot><mrow>" << parseMathML(content) << "</mrow>"
                               << "<mn>" << index << "</mn></mroot>";
                        }
                        continue;
                    }
                }
                
                // Unknown - skip
                i = cmdEnd;
                continue;
            }
            
            // Superscript
            if (input[i] == '^') {
                // Need to wrap previous element
                i++;
                std::string sup;
                if (i < input.length() && input[i] == '{') {
                    sup = extractBraced(input, i);
                } else if (i < input.length()) {
                    sup = input[i++];
                }
                ss << "<msup><mrow></mrow><mrow>" << parseMathML(sup) << "</mrow></msup>";
                continue;
            }
            
            // Subscript
            if (input[i] == '_') {
                i++;
                std::string sub;
                if (i < input.length() && input[i] == '{') {
                    sub = extractBraced(input, i);
                } else if (i < input.length()) {
                    sub = input[i++];
                }
                ss << "<msub><mrow></mrow><mrow>" << parseMathML(sub) << "</mrow></msub>";
                continue;
            }
            
            // Numbers
            if (std::isdigit(input[i]) || input[i] == '.') {
                std::string num;
                while (i < input.length() && (std::isdigit(input[i]) || input[i] == '.')) {
                    num += input[i++];
                }
                ss << "<mn>" << num << "</mn>";
                continue;
            }
            
            // Letters (variables)
            if (std::isalpha(input[i])) {
                ss << "<mi>" << input[i++] << "</mi>";
                continue;
            }
            
            // Operators
            if (input[i] == '+' || input[i] == '-' || input[i] == '*' || 
                input[i] == '/' || input[i] == '=' || input[i] == '<' ||
                input[i] == '>' || input[i] == '(' || input[i] == ')' ||
                input[i] == '[' || input[i] == ']' || input[i] == '|' ||
                input[i] == ',' || input[i] == '!') {
                ss << "<mo>" << input[i++] << "</mo>";
                continue;
            }
            
            // Skip whitespace
            if (std::isspace(input[i])) {
                i++;
                continue;
            }
            
            // Unknown - output as-is
            ss << input[i++];
        }
        
        return ss.str();
    }
    
    /**
     * @brief Escape special XML characters
     */
    static std::string escapeXML(const std::string& text) {
        std::string result;
        for (char c : text) {
            switch (c) {
                case '<': result += "&lt;"; break;
                case '>': result += "&gt;"; break;
                case '&': result += "&amp;"; break;
                case '"': result += "&quot;"; break;
                case '\'': result += "&apos;"; break;
                default: result += c;
            }
        }
        return result;
    }
};

/**
 * @brief Helper function to render LaTeX to Unicode
 */
inline std::string latex(const std::string& expr) {
    return LaTeXRenderer::toUnicode(expr);
}

/**
 * @brief Helper function to check if text needs LaTeX rendering
 */
inline bool needsLatex(const std::string& text) {
    return LaTeXRenderer::hasLaTeX(text);
}

} // namespace cppplot

#endif // CPPPLOT_CORE_LATEX_HPP
