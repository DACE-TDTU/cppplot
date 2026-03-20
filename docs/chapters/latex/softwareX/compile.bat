@echo off
echo Compiling SoftwareX LaTeX Manuscript...
cd manuscript

pdflatex -interaction=nonstopmode main.tex
bibtex main
pdflatex -interaction=nonstopmode main.tex
pdflatex -interaction=nonstopmode main.tex

echo Done! Verify main.pdf in manuscript directory.
cd ..
