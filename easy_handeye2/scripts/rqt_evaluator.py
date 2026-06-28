#!/usr/bin/env python

import sys

from rqt_gui.main import Main

main = Main()
argv = list(sys.argv)
if '--force-discover' not in argv:
    argv.extend(['--', '--force-discover'])
sys.exit(main.main(argv, standalone='easy_handeye2.handeye_rqt_evaluator.RqtHandeyeEvaluator'))
