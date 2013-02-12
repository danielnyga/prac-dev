( echo "Creating DRS to ACE independent executable ..." ;
swipl -O -f DRStoACEwrapper.pl -g "qsave_program('drsverb', [goal(main), toplevel(halt), local(25000), global(50000)])." -t halt 
echo "... done." 
);