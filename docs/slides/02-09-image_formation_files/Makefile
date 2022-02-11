OO=out
all: main.pdf

main.pdf: $(OO)/main.pdf
	cp $< $@

media/%:
	$(MAKE) -C media/ $*

include pdflatex.mk
