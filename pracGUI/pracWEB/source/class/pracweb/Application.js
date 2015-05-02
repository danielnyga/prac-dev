/* ************************************************************************

   Copyright:

   License:

   Authors: Daniel Nyga, Mareike Picklum

************************************************************************ */

/* ************************************************************************


************************************************************************ */

/**
 * This is the main application class of your custom application "pracWEB"
 *
 * @asset(pracweb/*)
 */
qx.Class.define("pracweb.Application",
{
  extend : qx.application.Inline,



  /*
  *****************************************************************************
     MEMBERS
  *****************************************************************************
  */

  members :
  {
    /**
     * This method contains the initial application code and gets called 
     * during startup of the application
     * 
     * @lint ignoreDeprecated(alert)
     */


    main : function()
    {
      // Call super class
      this.base(arguments);

      var that = this;
	  
      // Enable logging in debug variant
      if (qx.core.Environment.get("qx.debug"))
      {
        // support native logging capabilities, e.g. Firebug for Firefox
        qx.log.appender.Native;
        // support additional cross-browser console. Press F7 to toggle visibility
        qx.log.appender.Console;
      }
      
	  // destroy the session before leaving PRAC
	  window.onbeforeunload = function () {
	  	// getSession().invalidate();
	  var	req = new qx.io.request.Xhr(); 
		req.setUrl("/_destroy_session");
		req.setMethod("POST");
		req.addListener("success", function(e) { 
			var tar = e.getTarget();								
			var response = tar.getResponse();
		});
		req.send();
	  }; 

      var contentIsle = new qx.ui.root.Inline(document.getElementById("container", true, true));
      contentIsle.setWidth(window.innerWidth);
      contentIsle.setHeight(.9*window.innerHeight);
      window.addEventListener("resize", function() {
      	contentIsle.setWidth(window.innerWidth);
      	contentIsle.setHeight(.9*window.innerHeight);
      });
      contentIsle.setLayout(new qx.ui.layout.Grow());
	
	
      // new container
      var container = new qx.ui.container.Composite(new qx.ui.layout.VBox()).set({
        padding: 0
      });

      var splitPane = new qx.ui.splitpane.Pane("horizontal");

      this.__pane = splitPane;

      // Create container with fixed dimensions for the left:
      var left = new qx.ui.container.Composite(new qx.ui.layout.VBox()).set({
        minWidth: 370
      });

      // Create container for the right:
      var innerSplitPane = new qx.ui.splitpane.Pane("horizontal");
      var main = new qx.ui.container.Composite(new qx.ui.layout.Grow()).set({
        minWidth: .55*window.innerWidth,
        minHeight: .9*window.innerHeight
      });

      var right = new qx.ui.container.Composite(new qx.ui.layout.Grow()).set({
        maxWidth: 444
      });

      var flowChartEmbed = new qx.ui.embed.Html();
      this._flowChartEmbed = flowChartEmbed;
      right.add(flowChartEmbed);
      this._load_flow_chart();

      // left
  	  var form = this.buildForm();

  	  // placeholder
      var placeholder = new qx.ui.container.Composite();
      placeholder.setHeight(80);
      left.add(placeholder);
      left.add(form);

      // visualization of svg graph
      var vizEmbedGrp = new qx.ui.groupbox.GroupBox("Visualization");
      var vizLayout = new qx.ui.layout.Grow();
      vizEmbedGrp.setLayout(vizLayout);
      var vizHTML = "<div id='viz' style='width: 100%; height: 100%;'></div>";
      var vizEmbed = new qx.ui.embed.Html(vizHTML);
      vizEmbedGrp.add(vizEmbed);

      // main
      main.add(vizEmbedGrp);
      this._left = left;
      this._main = main;
      this._right = right;

      innerSplitPane.add(main, 1);
      innerSplitPane.add(right, 2);

      splitPane.add(left, 0);
      splitPane.add(innerSplitPane, 1);

      var mainPane = this.buildMainPane();
      this._mainPane = mainPane;

      container.add(splitPane, {flex: 1}); //, { height : "auto" }
      container.add(mainPane);

      // add container to content div
      contentIsle.add(container);
      
      // initially do not show form and do NOT use stepwise inference by default
      this._showLeft = false;
      this._showRight = true;
      this.__stepwise = false;
      this._changeVisiblity();
    },


    loadGraph : function() {
      if (typeof this.__graph === 'undefined') {
        this.__graph = new pracweb.Graph();
      } 
      this.__graph.clear();
    },

    updateGraph : function(data) {
      this.__graph.updateData(data);
    },

    buildMainPane : function()
    {
      var mainGroup = new qx.ui.groupbox.GroupBox("PRAC Inference");

      var mainLayout = new qx.ui.layout.HBox(20);
      mainGroup.setLayout(mainLayout);
      
      var description = new qx.ui.form.TextField("");
      description.setMinWidth(300);
      
      var expSettings = new qx.ui.form.CheckBox("Use expert settings");
      expSettings.addListener("changeValue", function(e) {
        this._showLeft = e.getData();
        this._changeVisiblity();
      }, this);


      var stepInf = new qx.ui.form.CheckBox("Step-by-step inference");
      var dieter = new qx.ui.form.CheckBox("Show Dieter");
      dieter.addListener("changeValue", function(e) {
        this._showRight = e.getData();
        this._changeVisiblity();
      }, this);


      var vizButton = new qx.ui.form.Button("Run Inference", "/prac/static/images/resultset_next.png");
      this.__vizButton = vizButton;
      var nextButton = new qx.ui.form.Button("Next Step",  "/prac/static/images/resultset_last.png");
      this.__nextButton = nextButton;
      nextButton.setEnabled(false);
      
      /**
       * Taxonomy visualization
       */
      var wordnetButton = new qx.ui.form.Button("Show Taxonomy");
  	  // var wnWindow = qx.ui.window.Window("WordNet Taxonomy Visualization");
	  // wnWindow.setWidth(80);
	  // wnWindow.setHeight(60);
	  // this.getRoot().add(wnWindow, {left:20, top:20});
  	  // wnWindow.open();
  	  var win = new qx.ui.window.Window("First Window");
    	win.setWidth(300);
    	win.setHeight(200);
    	win.setShowMinimize(false);
    	win.setLayout(new qx.ui.layout.Grow());
    	var taxCanvas = new qx.ui.embed.Html();
	  	win.add(taxCanvas);
    	this.getRoot().add(win, {left:20, top:20});
    	// win.open();
  	  
  	  
      wordnetButton.addListener("execute", function() {
    		var req = new qx.io.request.Xhr(); 
    		req.setUrl("/_get_wordnet_taxonomy");
    		req.setMethod("GET");
  	  	
    		req.addListener("success", function(e) {
    			console.log("1");
    			var tar = e.getTarget();								
    			var response = tar.getResponse();
    			console.log("2");
  		  	console.log("3");
  		  	
  		  	console.log("4");
    			console.log(response);
  		  	taxCanvas.setHtml(response);
  		  	console.log(taxCanvas.getContentElement());
    			win.open();
  		  });
    		console.log("sending request");
    		req.send();
      }, this);
	  
    /* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
     * Trigger the PRAC inference
     */
      vizButton.addListener('execute', function() {
        if (this.__stepwise) {
          this.__vizButton.setEnabled(false);
        }
         this.loadGraph();
         this._clearFlowChart();
         document.getElementById('init').nextElementSibling.style.fill = "#bee280";
         var req = this._run_inference("POST");
         console.log(description.getValue());
         req.setRequestHeader("Content-Type", "application/json");
         req.setRequestData({"sentence": description.getValue()});
         req.send();
      }, this);
      
    /* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
     * Toggling Step-by-step check box will enable/disable 'Next'-Button and
     * set the inference type
     */
      stepInf.addListener("changeValue", function(e) {
        var that = this;
        nextButton.setEnabled(e.getData());
        that.__stepwise = e.getData();
      }, this);


    /* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
     * 'Next'-Button will trigger new inference step
     */
      nextButton.addListener('execute', function(e) {
        var that = this;
        var req = that._run_inference("GET");
            req.send();
      }, this);


      var getmodule = new qx.ui.form.Button("Get next module");
      getmodule.addListener('execute', this._get_next_module, this);

      mainGroup.add(new qx.ui.basic.Label("Natural-language instruction:"));
      mainGroup.add(description);
      mainGroup.add(expSettings);
      mainGroup.add(stepInf);
      mainGroup.add(dieter);
      mainGroup.add(vizButton);
      mainGroup.add(nextButton);
      mainGroup.add(wordnetButton);
      mainGroup.add(getmodule);
      return mainGroup;
    },

    _run_inference : function(method) {
      console.log('_run_inference');
      this._update_flowchart();
      
      

    	var req = new qx.io.request.Xhr(); 
  		req.setUrl("/_pracinfer_step");
  		req.setMethod(method);
  		req.setRequestHeader("Cache-Control", "no-cache");
  		var that = this;
  		req.addListener("success", function(e) {
  			var tar = e.getTarget();								
  			var response = tar.getResponse();
  			console.log(response.result);
  			console.log(response.finish);
  			if (response.finish) {
          //TODO: Show that inference is done, highlight result?
          that.updateGraph(response.result);
          console.log(" I am DONE! ");
          that.__nextButton.setEnabled(false);
          setTimeout( function() {
              that._clearFlowChart();
            }, 3000); // wait 3 seconds, then clear flowchart
  				return;
        }	else {
          that.updateGraph(response.result);
          if (!that.__stepwise) {
            console.log('bumming around for', response.result.length * 1000, ' mseconds before sending new request...');
            setTimeout( function() {
              var req = that._run_inference("GET");
              console.log("sending new request...");
              req.send();
            }, response.result.length * 1000); // wait for graph to be updated
          }
  			}
  		});
		  return req;
    },

    buildForm : function()
    {
      // build form
      var group = new qx.ui.groupbox.GroupBox("Expert Settings");
      var grouplayout = new qx.ui.layout.HBox();
      group.setLayout(grouplayout);
      var formLayout = new qx.ui.layout.Grid();
      formLayout.setRowFlex(0, 1); // make row 0 flexible
      formLayout.setRowFlex(1, 1); // make row 1 flexible
      formLayout.setColumnWidth(2, 70); // set with of column 1 to 200 pixel


      var formgroup = new qx.ui.container.Composite(formLayout);

      // create form widgets
      var module = new qx.ui.form.SelectBox("Select Module");
      var logic = new qx.ui.form.SelectBox("Select Logic");
      var kb = new qx.ui.form.SelectBox("Select KB");
      var mln_dd = new qx.ui.form.SelectBox("Select MLN");
      var mlnFile = new qx.ui.form.Button("Browse");
      var mln = new qx.ui.form.TextArea("");
      var evidence_dd = new qx.ui.form.SelectBox("Select Evidence");
      var evidenceFile = new qx.ui.form.Button("Browse");
      var evidence = new qx.ui.form.TextArea("");
      var method = new qx.ui.form.SelectBox("Select Method");
      var queries = new qx.ui.form.TextField("");
      var parameters = new qx.ui.form.TextField("");
      var cwPreds = new qx.ui.form.TextField("");
      var closedWorld = new qx.ui.form.CheckBox("Apply CW assumption");
      var useMultiCPU = new qx.ui.form.CheckBox("use all CPU's");
      var submitForm = new qx.ui.form.Button("Infer");
      var uploadMLNFile = new qx.ui.form.Button("upload");
      var uploadDBFile = new qx.ui.form.Button("upload");

      // listeners
      module.addListener("changeSelection", this._changeModule, this);

      // widget settings
      module.add(new qx.ui.form.ListItem("-one-"));
      module.add(new qx.ui.form.ListItem("-two-"));
      logic.add(new qx.ui.form.ListItem("-choose-"));
      kb.add(new qx.ui.form.ListItem("-choose-"));
      mln_dd.add(new qx.ui.form.ListItem("-choose-"));
      evidence_dd.add(new qx.ui.form.ListItem("-choose-"));
      method.add(new qx.ui.form.ListItem("-choose-"));

      // addding form elements to grid
      formgroup.add(new qx.ui.basic.Label("Module"), {row: 0, column: 0});
      formgroup.add(module, {row: 0, column: 1});

      formgroup.add(new qx.ui.basic.Label("Logic"), {row: 1, column: 0});
      formgroup.add(logic, {row: 1, column: 1});

      formgroup.add(new qx.ui.basic.Label("KB"), {row: 2, column: 0});
      formgroup.add(kb, {row: 2, column: 1});

      formgroup.add(new qx.ui.basic.Label("MLN"), {row: 3, column: 0});
      formgroup.add(mln_dd, {row: 3, column: 1});
      var mlnGroup = new qx.ui.groupbox.GroupBox();
      var mlnLayout = new qx.ui.layout.HBox(20);
      mlnGroup.setLayout(mlnLayout);
      mlnGroup.add(mlnFile);
      mlnGroup.add(uploadMLNFile);
      formgroup.add(mlnGroup, {row: 4, column: 1});
      formgroup.add(mln, {row: 5, column: 1});

      formgroup.add(new qx.ui.basic.Label("Evidence"), {row: 6, column: 0});
      formgroup.add(evidence_dd, {row: 6, column: 1});
      var dbGroup = new qx.ui.groupbox.GroupBox();
      var dbLayout = new qx.ui.layout.HBox(20);
      dbGroup.setLayout(dbLayout);
      dbGroup.add(evidenceFile);
      dbGroup.add(uploadDBFile);
      formgroup.add(dbGroup, {row: 7, column: 1});
      formgroup.add(evidence, {row: 8, column: 1});

      formgroup.add(new qx.ui.basic.Label("Method"), {row: 9, column: 0});
      formgroup.add(method, {row: 9, column: 1});

      formgroup.add(new qx.ui.basic.Label("Queries"), {row: 10, column: 0});
      formgroup.add(queries, {row: 10, column: 1});
      
      formgroup.add(new qx.ui.basic.Label("Parameters"), {row: 11, column: 0});
      formgroup.add(parameters, {row: 11, column: 1});

      formgroup.add(new qx.ui.basic.Label("CW Preds"), {row: 12, column: 0});
      formgroup.add(cwPreds, {row: 12, column: 1});

      var checkboxGroup = new qx.ui.groupbox.GroupBox();
      var groupLayout = new qx.ui.layout.HBox(20);
      checkboxGroup.setLayout(groupLayout);

      checkboxGroup.add(closedWorld);
      checkboxGroup.add(useMultiCPU);

      formgroup.add(checkboxGroup, {row: 13, column: 1});

      group.add(formgroup);
      return group;
    },

    _load_flow_chart : function(e)
    {
      console.log('loading flowchart...');
      var req = new qx.io.request.Xhr(); 
      req.setUrl("/_load_flow_chart");
      req.setMethod('GET');
      req.setRequestHeader("Cache-Control", "no-cache");
      req.setRequestHeader("Content-Type", "text/plain");
      var that = this;
      console.log('5');
      req.addListener("success", function(e) {
        console.log('6');
        var tar = e.getTarget();
        var response = tar.getResponse();
        this._flowChartEmbed.setHtml(response);             
        return;
      }, that);
      req.send();
    },

    _clearFlowChart : function(e) {
      var nodes = ['init','nl_parsing','ac_recognition','senses_and_roles','executable','finished','achieved_by','roles_transformation'];
      for (var x = 0; x < nodes.length; x++) {
        document.getElementById(nodes[x]).nextElementSibling.style.fill = "white";
      }
    },

    _update_flowchart : function(e)
    {
      console.log('getting next module...');
      var req = new qx.io.request.Xhr(); 
      req.setUrl("/_pracinfer_get_next_module");
      req.setMethod('GET');
      req.setRequestHeader("Cache-Control", "no-cache");
      req.setRequestHeader("Content-Type", "text/plain");
      var that = this;
      req.addListener("success", function(e) {
        console.log('success');
        var tar = e.getTarget();
        var response = tar.getResponse();
        console.log('got response from server', response);
        that._clearFlowChart();
        document.getElementById(response).nextElementSibling.style.fill = "#bee280";
        if (response === 'senses_and_roles' || response === 'roles_transformation') {
          setTimeout( function() {
            that._clearFlowChart();
            document.getElementById('executable').nextElementSibling.style.fill = "#bee280";
          }, 2000); 
        }
        return response;
      }, that);
      req.send();
    },

    _changeModule : function(e)
    {
      // pass
    },

    _changeVisiblity : function(e)
    {
      if (this._showLeft)
        this._left.show();
      else 
        this._left.exclude();
      this._main.show();
      if (this._showRight) 
        this._right.show();
      else 
        this._right.exclude();
    }
  }
});
