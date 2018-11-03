TARGET := melfa.pdf melfa.html melfa.docx

FILES:= melfa.mdpp step-1.md

all: $(TARGET)

melfa.md: melfa.mdpp $(FILES)
	markdown-pp -o $@ $<

%.pdf: %.md
	pandoc $^ -o $@ --latex-engine=lualatex --template `pwd`/eisvogel.latex -V CJKmainfont=IPAexGothic

%.tex: %.md
	pandoc $^ -o $@ --latex-engine=lualatex --template `pwd`/eisvogel.latex -V CJKmainfont=IPAexGothic

%.html: %.md
	pandoc $^ -o $@

%.docx: %.md
	pandoc $^ -o $@

clean:
	rm -f $(TARGET) melfa.md
