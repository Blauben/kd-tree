# Configuration file for the Sphinx documentation builder.
#
# For the full list of built-in configuration values, see the documentation:
# https://www.sphinx-doc.org/en/master/usage/configuration.html

import os

# -- Project information -----------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#project-information

project = 'KDTree'
copyright = '2025, Ben Frauenknecht'
author = 'Ben Frauenknecht'
release = 'v1.0.0'

# -- General configuration ---------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#general-configuration

extensions = ['sphinx_rtd_theme', 'breathe', 'sphinx.ext.autodoc']

templates_path = ['_templates']
exclude_patterns = []

# Breathe Configuration

breathe_projects = {
    # "KDTree": xml_dir set by cli
    "KDTree": ""
}
breathe_default_project = "KDTree"



# -- Options for HTML output -------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#options-for-html-output

html_theme = 'sphinx_rtd_theme'
html_static_path = ['_static']

html_theme_options = {
    'collapse_navigation': False,
    'navigation_depth': 4,
    'sticky_navigation': True,
}

suppress_warnings = ['duplicate_declaration.cpp']