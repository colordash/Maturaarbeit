# Anpassen, wenn noetig
#Name der Hauptdatei, ohne Endung .tex
TEX=praesentation
#Name vom Programm, das die PDF-Datei anzeigen soll 
VIEWER=okular 

# Automatische Variablen
SRC=$(wildcard *.tex)
BIB=$(wildcard *.bib)
BIBFILE=$(BIB:.bib=)
SHELL := /bin/bash

all:	$(TEX).pdf

$(TEX).pdf:	$(SRC) $(BIB)

%.pdf:	%.tex
	pdflatex $< && pdflatex $<
show:	all
	$(VIEWER) $(TEX).pdf

git-clean:
	rm -f $(TEX).{aux,bbl,blg,dvi,log,ps,toc,out,lof,ind,idx,ilg,bcf,lol,run.xml,nav,snm} *~ 

clean:	git-clean
	rm -f $(TEX).pdf


