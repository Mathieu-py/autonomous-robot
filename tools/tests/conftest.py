"""Lets the tests import the tools, which are scripts rather than a package.
Run: python3 -m pytest tools/tests"""
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent.parent))
