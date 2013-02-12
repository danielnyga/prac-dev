% <module> ACE Interface

%@author Nicholas H. Kirk
%@version 28-12-2012
%%%TEMPORARY SANDBOX FILE FOR ACE PROLOG INTERFACING%%%

%usage: swipl -s ACEInterface.pl -g verbalization -t halt

 
% We point to the directory where APE modules and the lexicons are located.
:- assert(user:file_search_path(src, '..')).

:- use_module(actioncore(parser/ace_to_drs), [acetext_to_drs/8]).

:- style_check(-discontiguous).
:- use_module(actioncore(lexicon/clex)).
:- use_module(actioncore(lexicon/ulex)).
:- style_check(+discontiguous).

:- use_module(actioncore(utils/drs_to_ascii), [drs_to_ascii/2]).
:- use_module(actioncore(utils/drs_to_ace), [drs_to_ace/2]).
:- use_module(actioncore(utils/drs_to_coreace), [bigdrs_to_coreace/2]).
:- use_module(actioncore(utils/serialize_term), [serialize_term_into_atom/2]).
		   % +Term, -Atom
:- use_module(actioncore(utils/drs_to_sdrs), [drs_to_sdrs/2]).
:- use_module(actioncore(utils/is_wellformed), [is_wellformed/1]).

acetext('input').

verbalization :-
	acetext(AceText),
	read_file_to_codes(AceText, AceTextCodes, [encoding(utf8)]),
	acetext_to_drs(AceTextCodes, on, off, _Sentences1, _SyntaxTrees1, Drs1, Messages1, _DurationList1),
	drs_to_ascii(Drs1, DrsAscii1),
%	format("DRS = ~n~w~n", [Drs1]),
%	format("DRS = ~n~w~n", [DrsAscii1]),
	serialize_term_into_atom(Drs1,Str),
%	drs_to_sdrs(Drs1,Out),
	format("DRS clean = ~n~w~n", Drs1),
        drs_to_ascii(Drs1, Ascii1),
	drs_to_ascii([drs([],[(drs([A],[object(A,man,countable,na,eq,1)-1/2]),drs([B,C],[object(B,human,countable,na,eq,1)-1/5,predicate(C,be,A,B)-1/3]))])], Ascii2),
	format("DRS serialized = ~n~w~n", [drs([],[=>(drs([A],[object(A,man,countable,na,eq,1)-1/2]),drs([B,C],[object(B,human,countable,na,eq,1)-1/5,predicate(C,be,A,B)-1/3]))])]),
%works with [STRING FROM WEBSERVICE]
	format("DRS clean (pretty)= ~n~w~n", Ascii1),
	format("DRS serial (pretty) = ~n~w~n", Ascii2),
        drs_to_ace(drs([],[question(drs([A,B,C,D],[object(A,something,dom,na,na,na)-1/1,query(B,what)-1/5,property(C,flipped,pos)-1/3,predicate(D,be,A,C)-1/2,modifier_pp(D,from,B)-1/4]))]),Ace1),
	format("ACE = ~n~w~n", Ace1).
%	bigdrs_to_coreace(Str,Ace2),
%	format("ACE = ~n~w~n", Ace1),
%	format("ACE = ~n~w~n", Ace2).
%	bigdrs_to_coreace(Drs1, Ace),
%	bigdrs_to_coreace(drs(X),[Ace]),
%        format("output = ~n~w~n", drs(X)).
