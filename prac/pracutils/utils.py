from pracmln.mln.util import colorize


def prac_heading(s, upper=True, color='green'):
    b = colorize('+{}+'.format(''.ljust(len(s)+2, '=')), (None, color, True), True)
    t = colorize('| {} |'.format(s.upper() if upper else s), (None, color, True), True)
    return '\n{}\n{}\n{}\n'.format(b, t, b)