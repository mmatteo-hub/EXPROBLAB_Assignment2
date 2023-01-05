# Configuration file for the Sphinx documentation builder.
#
# For the full list of built-in configuration values, see the documentation:
# https://www.sphinx-doc.org/en/master/usage/configuration.html

import os
import subprocess
import sys
sys.path.insert(0, os.path.abspath('..'))

subprocess.call('doxygen Doxyfile.in', shell=True)

# -- Project information -----------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#project-information

project = 'EXPROBLAB_Assignment2'
copyright = '2023, Matteo Maragliano'
author = 'Matteo Maragliano'
release = '1.0'

# -- General configuration ---------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#general-configuration

extensions = [
	'sphinx.ext.autodoc',
	'sphinx.ext.doctest',
	'sphinx.ext.intersphinx',
	'sphinx.ext.todo',
	'sphinx.ext.coverage',
	'sphinx.ext.mathjax',
	'sphinx.ext.ifconfig',
	'sphinx.ext.viewcode',
	'sphinx.ext.githubpages',
	"sphinx.ext.napoleon",
	'sphinx.ext.inheritance_diagram',
	'breathe'
]

# -- Options for HTML output -------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#options-for-html-output

templates_path = ['_templates']
exclude_patterns = ['_build', 'Thumbs.db', '.DS_Store']
highlight_language = 'c++'
source_suffix = '.rst'
master_doc = 'index'
html_theme = 'sphinx_rtd_theme'
html_static_path = ['_static']
autodoc_default_options = {"members": True, "undoc-members": True, "private-members": True}

# -- Extension configuration -------------------------------------------------

# -- Options for intersphinx extension ---------------------------------------

# Example configuration for intersphinx: refer to the Python standard library. 
intersphinx_mapping = {'https://docs.python.org/': None}

# -- Options for todo extension ----------------------------------------------

# If true, `todo` and `todoList` produce output, else they produce nothing. 
todo_include_todos = True

# -- Options for breathe
breathe_projects = {
	"detect_marker": "docs/_build/xml/"
}
breathe_default_project = "detect_marker"
breathe_default_members = ('members', 'undoc-members')