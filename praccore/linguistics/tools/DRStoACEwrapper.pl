% <module> DRS to ACE high level interface for compiled encapsulation

%@author Nicholas H. Kirk
%@version 29-01-2013

%how to call: swipl -O -f DRStoACEwrapper.pl -g "qsave_program('drsverb', [goal(main), toplevel(halt), local(25000), global(50000)])." -t halt 
%how to test: ./drsverb "DRS ARGUMENT"

:- assert(user:file_search_path(linguistics, '..')).
:- use_module(linguistics(utils/drs_to_ace), [drs_to_ace/2]).
:- use_module(linguistics(utils/drs_to_ascii), [drs_to_ascii/2]).

:- use_module(linguistics(utils/drs_to_coreace), [bigdrs_to_coreace/2]).

lastArgv([Elem], Elem).
lastArgv([_|Tail], Elem) :- lastArgv(Tail, Elem).

main :-
        current_prolog_flag(argv, ArgList),
        lastArgv(ArgList,Param),
        term_to_atom(T, Param),
        call(bigdrs_to_coreace(T,Ace0)),
        format("~w", [Ace0]).
