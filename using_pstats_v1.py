import pstats
p = pstats.Stats("result2.out")
p.strip_dirs().sort_stats('tottime').print_stats(50)