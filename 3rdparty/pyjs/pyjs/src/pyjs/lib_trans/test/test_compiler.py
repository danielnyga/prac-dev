from UnitTest import UnitTest
from textwrap import dedent
import os.path
import sys
import traceback

lib_trans = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, lib_trans)
import pyparser as parser
import pycompiler as compiler



class CompilerTest(UnitTest):
    def _test_compile(self, code, codestr):
        code2 = dedent(code) + "\n"
        tree = compiler.parse(code2)
        self.assertEqual(str(tree.node), codestr)

    def test_statements(self):
        statements = [
            ("a + 1",
             "Stmt([Discard(Add(Name('a'), Const(1)))])"),
            
            ("a = 1",
             "Stmt([Assign([AssName('a', 'OP_ASSIGN')], Const(1))])"),
            
            ("def test(): pass",
             "Stmt([Function(None, 'test', (), (), False,"
             " False, None, Stmt([Pass()]))])"),
            
            ("with a as b: pass",
             "Stmt([With(Name('a'), AssName('b', 'OP_ASSIGN'),"
             " Stmt([Pass()]))])"),
            
            ("""
            def test(a, b=123, **kw):
                yield b
            """,
               "Stmt([Function(None, 'test', ['a', 'b', 'kw'],"
               " [Const(123)], False, True, None,"
               " Stmt([Discard(Yield(Name('b')))]))])"),
            
            ("""
            @dec
            class X(object):
                pass
            """,
               "Stmt([Class('X', [Name('object')], None,"
               " Stmt([Pass()]), Decorators([Name('dec')]))])"),
            
            ("""
            @dec()
            def test():
                pass
            """,
               "Stmt([Function(Decorators([CallFunc(Name('dec'), [], None, None)]),"
               " 'test', (), (), False, False, None, Stmt([Pass()]))])"),
            
            ("""
            try:
                1//0
            except:
                e = 1
            finally:
                f = 1
            """,
               "Stmt([TryFinally(TryExcept(Stmt([Discard(FloorDiv("
               "Const(1), Const(0)))]), "
               "[(None, None, Stmt([Assign([AssName('e', 'OP_ASSIGN')], "
               "Const(1))]))], None), Stmt([Assign([AssName('f', 'OP_ASSIGN')], "
               "Const(1))]))])"),
            
            ("""
            "doc"
            
            def test():
                "doc"
                pass
            """,
               "Stmt([Function(None, 'test', (), (), False, "
               "False, 'doc', Stmt([Pass()]))])"),
            
            ("""
            def g():
                a = 1
                def f(): return a + 2
                return f()
            result = g()
            """,
               "Stmt([Function(None, 'g', (), (), False, False, None, "
               "Stmt([Assign([AssName('a', 'OP_ASSIGN')], Const(1)), "
               "Function(None, 'f', (), (), False, False, None, "
               "Stmt([Return(Add(Name('a'), Const(2)))])), "
               "Return(CallFunc(Name('f'), [], None, None))])), "
               "Assign([AssName('result', 'OP_ASSIGN')], "
               "CallFunc(Name('g'), [], None, None))])"),
            
            #("""
            #list((i,j) for i in range(3) if i < 3
            #           for j in range(4) if j > 2)
            #""",
            #   "Stmt([Discard(CallFunc(Name('list'), [GenExpr(GenExprInner(Tuple([Name('i'), Name('j')]), [GenExprFor(AssName('i', 'OP_ASSIGN'), CallFunc(Name('range'), [Const(3)], None, None), [GenExprIf(Compare(Name('i'), [('<', Const(3))]))]), GenExprFor(AssName('j', 'OP_ASSIGN'), CallFunc(Name('range'), [Const(4)], None, None), [GenExprIf(Compare(Name('j'), [('>', Const(2))]))])]))], None, None))])"),
            
            ("""
            {1, 2, 3}
            {1, 2, 3,}
            """,
               'Stmt([Discard(Set([Const(1), Const(2), Const(3)])), '
               'Discard(Set([Const(1), Const(2), Const(3)]))])'),
            
            ("""
            {1:2, 2:3, 3:4}
            {1:2, 2:3, 3:4,}
            """,
               'Stmt([Discard(Dict([(Const(1), Const(2)), '
               '(Const(2), Const(3)), (Const(3), Const(4))])), '
               'Discard(Dict([(Const(1), Const(2)), (Const(2), Const(3)), '
               '(Const(3), Const(4))]))])'),
            
            #("""
            #{x for x in range(1, 4)}
            #""",
            #   "Stmt([Discard(SetComp(Name('x'), "
            #   "[ListCompFor(AssName('x', 'OP_ASSIGN'), "
            #   "CallFunc(Name('range'), "
            #   "[Const(1), Const(4)], None, None), [])]))])"),
            
            #("""
            #{x:x+1 for x in range(1, 4)}
            #""",
            #   "Stmt([Discard(DictComp(Name('x'), Add(Name('x'), Const(1)), "
            #   "[ListCompFor(AssName('x', 'OP_ASSIGN'), "
            #   "CallFunc(Name('range'), "
            #   "[Const(1), Const(4)], None, None), [])]))])"),
            
            ("""
            with Ctx(1) as tc, Ctx(2) as tc2:
                pass
            """,
               "Stmt([With(CallFunc(Name('Ctx'), [Const(1)], None, None), "
               "AssName('tc', 'OP_ASSIGN'), "
               "With(CallFunc(Name('Ctx'), [Const(2)], None, None), "
               "AssName('tc2', 'OP_ASSIGN'), Stmt([Pass()])))])"),
            
            ("""
            global x
            x = 1
            """,
               "Stmt([Global(['x']), "
               "Assign([AssName('x', 'OP_ASSIGN')], Const(1))])"),
            
            ("""
            print 'a', 'b'
            print 'a',
            print('a', 'b')
            """,
               "Stmt([Printnl([Const('a'), Const('b')], None, True), "
               "Print([Const('a')], None), "
               "Printnl([Tuple([Const('a'), Const('b')])], None, True)])"), 

            #("""
            #a = 1 + 1j
            #b = 1.2356
            #""",
            #   "Stmt([Assign([AssName('a', 'OP_ASSIGN')], "
            #   "Add(Const(1), Const(1j))), "
            #   "Assign([AssName('b', 'OP_ASSIGN')], Const(1.2356))])"), 
            
            ("""
            [1,2,3][1:2:3]
            """,
               "Stmt([Discard(Subscript(List([Const(1), Const(2), Const(3)]), "
               "'OP_APPLY', [Sliceobj([Const(1), Const(2), Const(3)])]))])"), 
            
            ("""
            x.a.b.c()().__z__
            """,
               "Stmt([Discard(Getattr(CallFunc(CallFunc"
               "(Getattr(Getattr(Getattr(Name('x'), 'a'), 'b'), 'c'), "
               "[], None, None), [], None, None), '__z__'))])"), 
            
            ("""
            # comment1
            class X():
                # comment2
                class Z():
                    # comment3
                    z = 1
                x = 2
            """,
               "Stmt([Class('X', [], None, "
               "Stmt([Class('Z', [], None, "
               "Stmt([Assign([AssName('z', 'OP_ASSIGN')], Const(1))]), None), "
               "Assign([AssName('x', 'OP_ASSIGN')], Const(2))]), None)])"),
        ]
            
        for code, codestr in statements:
            self._test_compile(code, codestr)

from RunTests import RunTests
def test_main():
    t = RunTests()
    t.add(CompilerTest)
    t.start_test()

if __name__ == '__main__':
    test_main()

