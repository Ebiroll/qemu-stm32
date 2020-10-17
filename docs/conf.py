# -*- coding: utf-8 -*-
#
# QEMU documentation build configuration file, created by
# sphinx-quickstart on Thu Jan 31 16:40:14 2019.
#
# This config file can be used in one of two ways:
# (1) as a common config file which is included by the conf.py
# for each of QEMU's manuals: in this case sphinx-build is run multiple
# times, once per subdirectory.
# (2) as a top level conf file which will result in building all
# the manuals into a single document: in this case sphinx-build is
# run once, on the top-level docs directory.
#
# QEMU's makefiles take option (1), which allows us to install
# only the ones the user cares about (in particular we don't want
# to ship the 'devel' manual to end-users).
# Third-party sites such as readthedocs.org will take option (2).
#
#
# This file is execfile()d with the current directory set to its
# containing dir.
#
# Note that not all possible configuration values are present in this
# autogenerated file.
#
# All configuration values have a default; values that are commented out
# serve to show the default.

import os
import sys
import sphinx
from sphinx.errors import ConfigError

# Make Sphinx fail cleanly if using an old Python, rather than obscurely
# failing because some code in one of our extensions doesn't work there.
# In newer versions of Sphinx this will display nicely; in older versions
# Sphinx will also produce a Python backtrace but at least the information
# gets printed...
if sys.version_info < (3,6):
    raise ConfigError(
        "QEMU requires a Sphinx that uses Python 3.6 or better\n")

# The per-manual conf.py will set qemu_docdir for a single-manual build;
# otherwise set it here if this is an entire-manual-set build.
# This is always the absolute path of the docs/ directory in the source tree.
try:
    qemu_docdir
except NameError:
    qemu_docdir = os.path.abspath(".")

# If extensions (or modules to document with autodoc) are in another directory,
# add these directories to sys.path here. If the directory is relative to the
# documentation root, use an absolute path starting from qemu_docdir.
#
# Our extensions are in docs/sphinx; the qapidoc extension requires
# the QAPI modules from scripts/.
sys.path.insert(0, os.path.join(qemu_docdir, "sphinx"))
sys.path.insert(0, os.path.join(qemu_docdir, "../scripts"))


# -- General configuration ------------------------------------------------

# If your documentation needs a minimal Sphinx version, state it here.
#
# Sphinx 1.5 and earlier can't build our docs because they are too
# picky about the syntax of the argument to the option:: directive
# (see Sphinx bugs #646, #3366).
needs_sphinx = '1.6'

# Add any Sphinx extension module names here, as strings. They can be
# extensions coming with Sphinx (named 'sphinx.ext.*') or your custom
# ones.
extensions = ['kerneldoc', 'qmp_lexer', 'hxtool', 'depfile', 'qapidoc']

# Add any paths that contain templates here, relative to this directory.
templates_path = ['_templates']

# The suffix(es) of source filenames.
# You can specify multiple suffix as a list of string:
#
# source_suffix = ['.rst', '.md']
source_suffix = '.rst'

# The master toctree document.
master_doc = 'index'

# General information about the project.
project = u'QEMU'
copyright = u'2020, The QEMU Project Developers'
author = u'The QEMU Project Developers'

# The version info for the project you're documenting, acts as replacement for
# |version| and |release|, also used in various other places throughout the
# built documents.

# Extract this information from the VERSION file, for the benefit of
# standalone Sphinx runs as used by readthedocs.org. Builds run from
# the Makefile will pass version and release on the sphinx-build
# command line, which override this.
try:
    extracted_version = None
    with open(os.path.join(qemu_docdir, '../VERSION')) as f:
        extracted_version = f.readline().strip()
except:
    pass
finally:
    if extracted_version:
        version = release = extracted_version
    else:
        version = release = "unknown version"

# The language for content autogenerated by Sphinx. Refer to documentation
# for a list of supported languages.
#
# This is also used if you do content translation via gettext catalogs.
# Usually you set "language" from the command line for these cases.
language = None

# List of patterns, relative to source directory, that match files and
# directories to ignore when looking for source files.
# This patterns also effect to html_static_path and html_extra_path
exclude_patterns = ['_build', 'Thumbs.db', '.DS_Store']

# The name of the Pygments (syntax highlighting) style to use.
pygments_style = 'sphinx'

# If true, `todo` and `todoList` produce output, else they produce nothing.
todo_include_todos = False

# Sphinx defaults to warning about use of :option: for options not defined
# with "option::" in the document being processed. Turn that off.
suppress_warnings = ["ref.option"]

# The rst_epilog fragment is effectively included in every rST file.
# We use it to define substitutions based on build config that
# can then be used in the documentation. The fallback if the
# environment variable is not set is for the benefit of readthedocs
# style document building; our Makefile always sets the variable.
confdir = os.getenv('CONFDIR', "/etc/qemu")
rst_epilog = ".. |CONFDIR| replace:: ``" + confdir + "``\n"
# We slurp in the defs.rst.inc and literally include it into rst_epilog,
# because Sphinx's include:: directive doesn't work with absolute paths
# and there isn't any one single relative path that will work for all
# documents and for both via-make and direct sphinx-build invocation.
with open(os.path.join(qemu_docdir, 'defs.rst.inc')) as f:
    rst_epilog += f.read()

# -- Options for HTML output ----------------------------------------------

# The theme to use for HTML and HTML Help pages.  See the documentation for
# a list of builtin themes.
#
html_theme = 'alabaster'

# Theme options are theme-specific and customize the look and feel of a theme
# further.  For a list of options available for each theme, see the
# documentation.
# We initialize this to empty here, so the per-manual conf.py can just
# add individual key/value entries.
html_theme_options = {
}

# Add any paths that contain custom static files (such as style sheets) here,
# relative to this directory. They are copied after the builtin static files,
# so a file named "default.css" will overwrite the builtin "default.css".
# QEMU doesn't yet have any static files, so comment this out so we don't
# get a warning about a missing directory.
# If we do ever add this then it would probably be better to call the
# subdirectory sphinx_static, as the Linux kernel does.
# html_static_path = ['_static']

# Custom sidebar templates, must be a dictionary that maps document names
# to template names.
#
# This is required for the alabaster theme
# refs: http://alabaster.readthedocs.io/en/latest/installation.html#sidebars
html_sidebars = {
    '**': [
        'about.html',
        'navigation.html',
        'searchbox.html',
    ]
}

# Don't copy the rST source files to the HTML output directory,
# and don't put links to the sources into the output HTML.
html_copy_source = False

# -- Options for HTMLHelp output ------------------------------------------

# Output file base name for HTML help builder.
htmlhelp_basename = 'QEMUdoc'


# -- Options for LaTeX output ---------------------------------------------

latex_elements = {
    # The paper size ('letterpaper' or 'a4paper').
    #
    # 'papersize': 'letterpaper',

    # The font size ('10pt', '11pt' or '12pt').
    #
    # 'pointsize': '10pt',

    # Additional stuff for the LaTeX preamble.
    #
    # 'preamble': '',

    # Latex figure (float) alignment
    #
    # 'figure_align': 'htbp',
}

# Grouping the document tree into LaTeX files. List of tuples
# (source start file, target name, title,
#  author, documentclass [howto, manual, or own class]).
latex_documents = [
    (master_doc, 'QEMU.tex', u'QEMU Documentation',
     u'The QEMU Project Developers', 'manual'),
]


# -- Options for manual page output ---------------------------------------
# Individual manual/conf.py can override this to create man pages
man_pages = []

# -- Options for Texinfo output -------------------------------------------

# Grouping the document tree into Texinfo files. List of tuples
# (source start file, target name, title, author,
#  dir menu entry, description, category)
texinfo_documents = [
    (master_doc, 'QEMU', u'QEMU Documentation',
     author, 'QEMU', 'One line description of project.',
     'Miscellaneous'),
]



# We use paths starting from qemu_docdir here so that you can run
# sphinx-build from anywhere and the kerneldoc extension can still
# find everything.
kerneldoc_bin = ['perl', os.path.join(qemu_docdir, '../scripts/kernel-doc')]
kerneldoc_srctree = os.path.join(qemu_docdir, '..')
hxtool_srctree = os.path.join(qemu_docdir, '..')
qapidoc_srctree = os.path.join(qemu_docdir, '..')
