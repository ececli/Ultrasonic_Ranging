import pstats
p = pstats.Stats("result.out")
p.strip_dirs().sort_stats(-1).print_stats()