% <module> ACE Interface

%@author Nicholas H. Kirk
%@version 28-12-2012
%%%TEMPORARY SANDBOX FILE FOR ACE PROLOG INTERFACING%%%

%usage: swipl -s ACEInterface.pl -g verbalization -t halt



% We point to the directory where APE modules and the lexicons are located.
:- assert(user:file_search_path(linguistics, '..')).

:- use_module(linguistics(parser/ace_to_drs), [acetext_to_drs/8]).

:- style_check(-discontiguous).
:- use_module(linguistics(lexicon/clex)).
:- use_module(linguistics(lexicon/ulex)).
:- style_check(+discontiguous).

:- use_module(linguistics(utils/drs_to_ascii), [drs_to_ascii/2]).
:- use_module(linguistics(utils/drs_to_ace), [drs_to_ace/2]).
:- use_module(linguistics(utils/drs_to_coreace), [bigdrs_to_coreace/2]).
:- use_module(linguistics(utils/serialize_term), [serialize_term_into_atom/2]).
		   % +Term, -Atom
:- use_module(linguistics(utils/drs_to_sdrs), [drs_to_sdrs/2]).
:- use_module(linguistics(utils/is_wellformed), [is_wellformed/1]).



verbalization :-
	format("DRS = ~n~w~n", drs([],[=>(drs([A],[object(A,man,countable,na,eq,1)-1/2]),drs([B,C],[object(B,human,countable,na,eq,1)-1/5,predicate(C,be,A,B)-1/3]))]) ),
        drs_to_ascii( drs([],[=>(drs([A],[object(A,man,countable,na,eq,1)-1/2]),drs([B,C],[object(B,human,countable,na,eq,1)-1/5,predicate(C,be,A,B)-1/3]))]) , Ascii1),
	format("DRS clean (pretty)= ~n~w~n", Ascii1),
        bigdrs_to_coreace( drs([],[=>(drs([A],[object(A,man,countable,na,eq,1)-1/2]),drs([B,C],[object(B,human,countable,na,eq,1)-1/5,predicate(C,be,A,B)-1/3]))]) ,Ace1),
	format("ACE = ~n~w~n", Ace1).

