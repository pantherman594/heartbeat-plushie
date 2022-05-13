#!/bin/sh

source .venv/bin/activate

cp code.py code.min.py
autoflake --in-place --remove-unused-variables --remove-all-unused-imports code.min.py
pyminify code.min.py --in-place --rename-globals --remove-literal-statements
