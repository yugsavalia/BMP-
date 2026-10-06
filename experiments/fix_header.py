"""Prepend the CSV header to a headerless run_grid output. usage: python fix_header.py w4.csv"""
import os, sys; sys.path[:0] = [os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", d) for d in ("sim", "experiments", "analysis")]  # repo-local imports
import sys
from run_grid import FIELDS
p = sys.argv[1]; body = open(p).read()
if not body.startswith("layout,"):
    open(p, "w", newline="").write(",".join(FIELDS) + "\r\n" + body)
    print("header added")
else:
    print("already has header")