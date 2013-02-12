% <module> DRS to ACE high level interface for compiled encapsulation

%@author Nicholas H. Kirk
%@version 29-01-2013

%how to call: swipl -O -f DRStoACEwrapper.pl -g "qsave_program('drsverb', [goal(main), toplevel(halt), local(25000), global(50000)])." -t halt 
%how to test: swipl

:- assert(user:file_search_path(linguistics, '..')).
:- use_module(linguistics(utils/drs_to_ace), [drs_to_ace/2]).
%:- use_module(linguistics(utils/drs_to_coreace), [bigdrs_to_coreace/2]).


lastArgv([Elem], Elem).
lastArgv([_|Tail], Elem) :- lastArgv(Tail, Elem).

main :-
       	current_prolog_flag(argv, ArgList),
        lastArgv(ArgList,Param),
	term_to_atom(T, Param),
	call(drs_to_ace(T,Ace0)),
        format("DRS from argv = ~n~w~n", [Ace0]),
        drs_to_ace(drs([],[=>(drs([A],[object(A,man,countable,na,eq,1)-1/2]),drs([B,C],[object(B,human,countable,na,eq,1)-1/5,predicate(C,be,A,B)-1/3]))]),Ace1),
        format("ACE Hardcoded = ~n~w~n", [Ace1]),
        drs_to_ace(Drs,Ace),
        format("ACE Translated = ~n~w~n", [Ace]).               


%Produces the following output:
%nick@nick-XPS:~/Workspace/PRAC/mturk/edu.tum.cs.prac/src/linguistics/tools$ ./drsverb "drs([],[=>(drs([A],[object(A,man,countable,na,eq,1)-1/2]),drs%([B,C],[object(B,human,countable,na,eq,1)-1/5,predicate(C,be,A,B)-1/3]))])"
%DRS from argv = 
%drs([],[=>(drs([A],[object(A,man,countable,na,eq,1)-1/2]),drs([B,C],[object(B,human,countable,na,eq,1)-1/5,predicate(C,be,A,B)-1/3]))])
%ACE Hardcoded = 
%[[Every man is a human.]]


%ACE Translated missing!!

