import re
import os
import sys
import time
today = time.strftime("%d/%m/%Y")
report = open("../doc/g24_project_report.tex","r")
output = open("../doc/g24_project_report.html","w")
remove = ("\documentclass","\usepackage",r"\makeat",r"\renewcomma",r"\ifnum",r"\fi",r"\else",r"\bibliography",r"\maketitle")
output.write("<html>\n")
firstsec = True
firstsubsec = True
subseccon = False
newsec = True
authors = ""
authorscope = False
and_found = False
open_brace = False
close_brace = False
lstlisting = False
drop = False
for line in report:
    if line.find(r"\section")!=-1:
        drop = False
    if drop:
        continue
    if line.find(r"\end{lstlisting}")!=-1:
        lstlisting = False
        continue
    if lstlisting:
	line = line + "<br>\n"
        output.write(line)
        continue
    if line.find(r"\author")!=-1:
        authorscope = True
        open_brace = True
        output.write("<div style='margin-left:10%'>\n")
        continue
    if authorscope:

        if line.find(r"}")!=-1:
            close_brace = True
        if line.find(r"\and")!=-1:
            and_found = True
            continue
        if and_found: 
            line = "</div>\n" + "<div style='margin-left:5%;margin-right:5%;float:left'>\n" + line
            and_found = False
        if open_brace:
            line = "<div style='margin-left:5%;margin-right:5%;float:left'>\n" + line
            open_brace = False
        if close_brace:
            line = "</div>\n" + "<br style='clear: left' />\n" + "</div>\n"
            close_brace = False
            authorscope = False     
    if line.find(r"\begin")!=-1:
        if line.find(r"{document}",line.find(r"\begin")+ 6)!=-1:
            line = re.sub("\\\\begin","",line)
            line = re.sub(r"{document}.*","<body style='margin-top:5%;margin-bottom:5%'>\n",line)
            line = line.strip()
        elif line.find(r"{center}")!=-1:
            line = re.sub("\\\\begin","",line)
            line = re.sub(r"{center}.*","<div style='align:center'>\n",line)
        elif line.find(r"{lstlisting}")!=-1:
            lstlisting = True
            continue
    elif line.find(r"\end")!=-1:
        if line.find(r"{document}",line.find(r"\end")+ 4)!=-1:
            line = re.sub("\\\\end","",line)
            line = "</div>\n" + line + "\n"
            line = re.sub(r"{document}.*","</body>\n",line)
            line = line.strip()
        elif line.find(r"{center}")!=-1:
            line = re.sub("\\\\end","",line)
            line = re.sub(r"{center}.*","</div>\n",line)
    elif line.find(r"\section")!=-1:
        drop = False
        firstsubsec = True
        a = line.find(r"{")
        b = line.find(r"}")
        heading = line[a+1:b]
        if line.find(r"Profiling the ")!=-1:
            drop = True
            continue
        if firstsec:
            line =  "<div style='margin-left:10%;margin-right:10%'>\n" + "<h2>" + heading + "</h2>\n"
            firstsec = False
            subseccon = False
        else:
            if not subseccon:
                line = "</div>\n" +  "<div style='margin-left:10%;margin-right:10%'>\n" +"<h2>" + heading + "</h2>\n"
            else:
                line = "</div>\n" + "</div>\n" + "<div style='margin-left:10%;margin-right:10%'>\n" +"<h2>" + heading + "</h2>\n"
                subseccon = False
    elif line.find(r"\subsection")!=-1:
        subseccon = True
        a = line.find(r"{")
        b = line.find(r"}")
        heading = line[a+1:b]
        if firstsubsec:
            line =  "<div style='margin-left:0%;margin-right:0%'>\n" + "<h3>" + heading + "</h3>\n"
            firstsubsec = False
        else:
            line = "</div>\n" +  "<div style='margin-left:0%;margin-right:0%'>\n" +"<h3>" + heading + "</h3>\n"
    elif line.find(r"\includegraphics") != -1:
        a = line.find(r"{")
        b = line.find(r"}")
        path = line[a+1:b]
        line = "<img src='" + path + "' width=90%>\n"
        if path.find("output")!=-1:
            line = "<img src='" + path + "' width=90%>\n"
        
    elif line.find(r"\title")!=-1:
        a = line.find(r"{")
        b = line.find(r"}")
        title = line[a+1:b]
        line = "<h1 style='text-align:center'>" + title + "</h1>\n"
    elif line.find(r"\\")!=-1:
        line = re.sub("\\\\\\\\","<br>",line)
    elif line.find(r"\newline")!=-1:
        #line = re.sub("\\"," ",line)
        line = re.sub("\\\\newline","<br>",line)
	
    elif line.find(r"\date")!=-1:
        line = today
        line = "<br>\n<center>\n" + line + "</center>\n"
    elif line.find(r"\cite")!=-1:
	line = ""
    line = re.sub("\\\\%","%",line)
    line = re.sub("\\\\&","&",line)
    line = re.sub("\\\\_","_",line)
    if not any(s in line for s in remove):
        output.write(line)
output.write("</html>\n")
