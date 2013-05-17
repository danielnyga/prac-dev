Pyjs
=======

Pyjs is a port of Google Web Toolkit to Python, and thus enables
the development of Rich Media AJAX applications in Python, with no
need for special browser plugins.  Pyjs contains a stand-alone
python-to-javascript compiler, and also a Widget Set API that looks
very similar to Desktop Widget Set APIs (such as PyQT4 or PyGTK2).

Pyjs also contains a Desktop Widget Set version, running as pure
python, with three useable Desktop ports available.  With web-engine
technology at it's core, HTML, CSS, plugins and other related features
work out-of-the-box.

For more information and documentation, see:

* http://pyjs.org
* https://github.com/pyjs/pyjs/wiki

For issue tracking:

* https://github.com/pyjs/pyjs/issues

Mailing list:

* https://groups.google.com/group/pyjs-users

IRC:

:Server: irc.freenode.net
:Channel: #pyjs

Pyjs-Desktop
===============

Pyjs runs your python application in a Web Browser (as javascript).
Pyjs-Desktop runs exactly the same python application on the
Desktop (as pure python).  There are currently three engine
choices, with alternatives in development.

All ports of Pyjs-Desktop require a JSON library: simplejson is
recommended if the version of python is 2.5 or less.  Python 2.6
and above come with a json library installed by default.

1. XULRunner

   Install hulahop and python-xpcom.  Hulahop, from OLPC SugarLabs,
   is distributed with both Debian and Ubuntu; python-xpcom is part
   of XULRunner and is also distributed with both Debian and Ubuntu.
   Other users should investigate the installation instructions for
   python-xpcom and hulahop for the operating system of their choice
   on the appropriate web sites.

   Older versions of XULRunner must be used: versions 1.9.0 or 1.9.1 are
   known to be suitable, as is version 9.0.  Versions 10 and above are
   known to segfault.

2. PyWebKitGtk

   The version of pywebkitgtk at http://www.gnu.org/software/pythonwebkit
   provides full and direct python-equivalent interoperability for all functions
   for which access through javascript has been provided: thus, Pyjs
   Desktop will function correctly.

   PyWebkitGtk must be explicitly enabled.  create a $HOME/.pyjd/pyjdrc file
   containing the following two lines:

   [gui]
   engine=pywebkitgtk

3. PyWebkitDFB

   This is an experimental but minimally functional engine that is extremely
   quick to start up.  The build dependencies are also drastically smaller than
   any of the other web browser engines (which indirectly contributes to the
   fast startup time).

   HTML5 is fully supported, with the exception of Video and Canvas; also
   missing at present is support for Frames.  Despite the present limitations,
   PyWebkitDFB is highly suited to embedded systems, as well as being useable
   as an excellent and ultra-quick general-purpose web browser engine.

   PyWebkitDFB must be explicitly enabled.  create a $HOME/.pyjd/pyjdrc file
   containing the following two lines:

   [gui]
   engine=pywebkitdfb

4. MSHTML

   For Windows users, all that's required, other than installing python
   and Internet Explorer, is one further tiny package: Win32 "comtypes".

   Win32 "comtypes" can be downloaded here:
   * http://sourceforge.net/projects/comtypes/

   Unlike the other ports, which can comprise a whopping great bundle
   of anything up to 30mb in size, the MSHTML port literally requires
   nothing more than comtypes, thanks to the far-sighted design of the
   MSHTML Trident Engine and its extensive COM interface.
