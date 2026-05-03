#!/usr/bin/env python3
"""
Convenient launcher for the application.

Usage:
  python run.py                                               # normal GUI
  python run.py --replay recordings/fixtures/synthetic_smooth.csv
  python run.py --record my_session.csv
  python run.py --replay foo.csv --record bar.csv            # replay + record simultaneously
  python run.py --no-filter                                  # bypass ComplementaryFilter
  python run.py --help                                       # full flag reference
"""

from frontend.app import main

if __name__ == "__main__":
    main()
