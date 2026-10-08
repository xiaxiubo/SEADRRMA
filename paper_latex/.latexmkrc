# LaTeXmk configuration for paper_dr_rma.tex
$pdf_mode = 1;        # Use pdflatex
$bibtex_use = 2;      # Run bibtex when needed
$pdflatex = 'pdflatex -interaction=nonstopmode -synctex=1 %O %S';
$out_dir = '.';       # Output in current directory
$clean_ext = 'synctex.gz synctex.gz(busy) run.xml tex.bak bbl bcf fdb_latexmk run tdo';
