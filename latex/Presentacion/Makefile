DOCUMENTO = slides
TEMPORALES = *idx *aux *lof *log *lot *toc *bbl *blg *~ *out *rel *spl *loa *brf *nav *snm *vrb
HOY=$(shell date +"%Y-%m-%d")

all: pdf 

dvi: ${DOCUMENTO}.tex
	latex ${DOCUMENTO}

ps: dvi
	dvips -o ${DOCUMENTO}.ps ${DOCUMENTO}.dvi

pdf: 
	pdflatex ${DOCUMENTO}
	pdflatex ${DOCUMENTO}
	rm -f $(TEMPORALES) *dvi

backup:
	tar -cvzf ${HOY}.tgz ${DOCUMENTO}.tex  ./figs/* 

clean:
	rm -f $(TEMPORALES)

cleanall:
	rm -f $(TEMPORALES) *pdf *dvi

release:
	tar -cvzf $(HOY).tgz *
