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
  		req.setUrl("/prac/_destroy_session");
  		req.setMethod("POST");
  		req.addListener("success", function(e) { 
  			var tar = e.getTarget();								
  			var response = tar.getResponse();
  		});
  		req.send();
  	  }; 

      var contentIsle = new qx.ui.root.Inline(document.getElementById("prac_container", true, true));
      contentIsle.setWidth(document.getElementById("container", true, true).offsetWidth);
      contentIsle.setHeight(document.getElementById("container", true, true).offsetHeight);
      window.addEventListener("resize", function() {
      	contentIsle.setWidth(document.getElementById("container", true, true).offsetWidth);
      	contentIsle.setHeight(document.getElementById("container", true, true).offsetHeight);
      });
      contentIsle.setLayout(new qx.ui.layout.Grow());
	
      // main container (contains outer splitpane and control panel)
      var container = new qx.ui.container.Composite(new qx.ui.layout.VBox()).set({
        padding: 0
      });

      // outer splitpane (contains expert settings and inner splitpane)
      var splitPane = new qx.ui.splitpane.Pane("horizontal");
      this._pane = splitPane;

      // container for expert settings form
      var expSettingsContainer = new qx.ui.container.Composite(new qx.ui.layout.VBox()).set({
        minWidth: 370
      });
      this._expSettingsContainer = expSettingsContainer;

      // Create container for the right:
      var innerSplitPane = new qx.ui.splitpane.Pane("horizontal");
      var graphVizContainer = new qx.ui.container.Composite(new qx.ui.layout.Grow()).set({
        minHeight: .8*document.getElementById("container", true, true).offsetHeight
      });
      this._graphVizContainer = graphVizContainer;

      // scrollable container for the flowchart
      var flowChartScroll = new qx.ui.container.Scroll().set({
        width: 444,
        maxWidth: 444,
        height: 503
      });

      var flowChartContainer = new qx.ui.container.Composite(new qx.ui.layout.VBox()).set({
        width: 444,
        minWidth: 444,
        height: 503,
        minHeight: 503
       });

      flowChartScroll.add(flowChartContainer);

      // embedding for flowchart svg
      var flowChartEmbed = new qx.ui.embed.Html();
      this._flowChartEmbed = flowChartEmbed;
      flowChartContainer.add(flowChartEmbed, {flex: 1});
      this._load_flow_chart();

      // expert settings
      var form = this.buildForm();

      // placeholder
      var placeholder = new qx.ui.container.Composite();
      placeholder.setHeight(80);
      expSettingsContainer.add(placeholder);
      expSettingsContainer.add(form);

      // visualization of svg graph
      var vizEmbedGrp = new qx.ui.groupbox.GroupBox("Visualization");
      var vizLayout = new qx.ui.layout.Grow();
      vizEmbedGrp.setLayout(vizLayout);
      var vizHTML = "<div id='viz' style='width: 100%; height: 100%;'></div>";
      var vizEmbed = new qx.ui.embed.Html(vizHTML);
      vizEmbedGrp.add(vizEmbed);

      graphVizContainer.add(vizEmbedGrp);
      this._flowChartContainer = flowChartContainer;

      innerSplitPane.add(graphVizContainer);
      innerSplitPane.add(flowChartScroll);

      splitPane.add(expSettingsContainer, 0);
      splitPane.add(innerSplitPane, 1);

      // control pane (containing text field, buttons and checkboxes used for inference)
      var controlPane = this.buildControlPane();
      this._controlPane = controlPane;

      container.add(splitPane, {flex: 1});
      container.add(controlPane);

      // add container to content div
      contentIsle.add(container);
      
      // initially do not show form and do NOT use stepwise inference by default
      this._showexpSettingsContainer = false;
      this._stepwise = false;
      this._changeVisiblity();
    },


    loadGraph : function() {
      if (typeof this._graph === 'undefined') {
        this._graph = new pracweb.Graph();
      } 
      this._graph.clear();
    },

    updateGraph : function(data) {
      this._graph.updateData(data);
    },

    buildControlPane : function()
    {
      var mainGroup = new qx.ui.groupbox.GroupBox("PRAC Inference");
      var mainLayout = new qx.ui.layout.HBox(20);
      mainGroup.setLayout(mainLayout);
      
      // var description = new qx.ui.form.TextField("");
      // description.setMinWidth(300);
      // combobox containing nl descriptions
      var description = new qx.ui.form.ComboBox();
      description.setWidth(500);
      description.add(new qx.ui.form.ListItem("start the centrifuge."));
      description.add(new qx.ui.form.ListItem("Add some water to the purine."));
      description.add(new qx.ui.form.ListItem("Add some arsenic_acid to the imidazole."));
      description.add(new qx.ui.form.ListItem("Add 5 drops of the lysergic_acid to the pyrimidine."));
      description.add(new qx.ui.form.ListItem("Add 1 liter of water to the chlorous_acid."));
      description.add(new qx.ui.form.ListItem("Neutralize the methacrylic_acid with 100 milliliters of cyanuramide."));
      description.add(new qx.ui.form.ListItem("Start with neutralizing the pyridine with 4 bits of the hydrofluoric_acid. "));
      description.add(new qx.ui.form.ListItem("Open the drawer."));
      description.add(new qx.ui.form.ListItem("Open the fridge."));
      description.add(new qx.ui.form.ListItem("Open the cupboard."));
      description.setValue(description.getChildrenContainer().getSelectables()[0].getLabel());
      
      description.addListener("keydown", function(e) {
        this._vizButton.setEnabled(true);
        this._clearFlowChart();
        document.getElementById('init').nextElementSibling.style.fill = "#bee280";
      }, this);
      
      var expSettings = new qx.ui.form.CheckBox("Use expert settings");
      expSettings.addListener("changeValue", function(e) {
        this._showexpSettingsContainer = e.getData();
        this._changeVisiblity();
      }, this);


    /**
     * toggling Step-by-step check box will enable/disable 'Next'-button and
     * set the inference type
     */
      var stepInf = new qx.ui.form.CheckBox("Step-by-step inference");
      stepInf.addListener("changeValue", function(e) {
        var that = this;
        that._stepwise = e.getData();
      }, this);

      var vizButton = new qx.ui.form.Button("Run Inference", "/prac/static/images/resultset_next.png");
      this._vizButton = vizButton;
      var nextButton = new qx.ui.form.Button("Next Step",  "/prac/static/images/resultset_last.png");
      this._nextButton = nextButton;
      nextButton.setEnabled(false);
      
    /**
     * taxonomy visualization
     */
      var win = new qx.ui.window.Window("First Window");
      win.setWidth(700);
      win.setHeight(500);
      win.setShowMinimize(false);
      win.setLayout(new qx.ui.layout.Grow());
      var taxCanvas = new qx.ui.embed.Html();
      win.add(taxCanvas);
      this.getRoot().add(win, {left:20, top:20});
      
      var wordnetButton = new qx.ui.form.Button("Show Taxonomy");
      wordnetButton.addListener("execute", function() {
        var req = new qx.io.request.Xhr(); 
        req.setUrl("/prac/_get_wordnet_taxonomy");
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
    
    /**
     * trigger the PRAC inference
     */
      vizButton.addListener('execute', function() {
        if (this._stepwise) {
          this._vizButton.setEnabled(false);
        }
         this.loadGraph();
         this._clearFlowChart();
         this._next_module = 'nl_parsing';
         document.getElementById('init').nextElementSibling.style.fill = "#bee280";
         var req = this._run_inference("POST");
         console.log(description.getValue());
         req.setRequestHeader("Content-Type", "application/json");
         req.setRequestData({"sentence": description.getValue()});
         req.send();
      }, this);
      


    /**
     * 'Next'-button will trigger new inference step
     */
      nextButton.addListener('execute', function(e) {
        var that = this;
        var req = that._run_inference("GET");
            req.send();
      }, this);


      var getRoleDist = new qx.ui.form.Button("Get Role Distributions");
      getRoleDist.setEnabled(false);
      this._getRoleDist = getRoleDist;
      getRoleDist.addListener('execute', this._get_role_distributions, this);

      mainGroup.add(new qx.ui.basic.Label("Natural-language instruction:"));
      mainGroup.add(description);
      // mainGroup.add(expSettings); // temporarily removed for openEase integration
      mainGroup.add(stepInf);
      mainGroup.add(vizButton);
      mainGroup.add(nextButton);
      mainGroup.add(wordnetButton);
      mainGroup.add(getRoleDist);
      return mainGroup;
    },

  /**
   * update flowchart and request next inference step
   */
    _run_inference : function(method) {
      this._nextButton.setEnabled(false);

      // update flowchart
      if (this._next_module === 'achieved_by' || (this._next_module === 'plan_generation') && this._last_module != 'plan_generation') {
        var that = this;
        var tmpNM = that._next_module; //because following timeout will cause _next_module to be overwritten too quickly
        that._clearFlowChart();
        document.getElementById('executable').nextElementSibling.style.fill = "#bee280";
        setTimeout( function() {
          that._clearFlowChart();
          document.getElementById(tmpNM).nextElementSibling.style.fill = "#bee280";
        }, 2000);
      } else {
        this._clearFlowChart();
        document.getElementById(this._next_module).nextElementSibling.style.fill = "#bee280";
      } 

      // request next inference result
      var req = new qx.io.request.Xhr(); 
      req.setUrl("/prac/_pracinfer_step");
      req.setMethod(method);
      req.setRequestHeader("Cache-Control", "no-cache");
      req.addListener("success", function(e) {
        var that = this;
        var tar = e.getTarget();                
        var response = tar.getResponse();
        console.log('response result:', response.result);
        console.log('response finish:', response.finish);

        if (response.finish) {
          console.log(" I am DONE! ");
          setTimeout( function() {
            that._get_cram_plan();
          }, response.result.length * 1000); // wait 3 seconds, then clear flowchart
          that.updateGraph(response.result);
          that._nextButton.setEnabled(false);
          that._vizButton.setEnabled(true);
          that._last_module = '';
        } else if (that._next_module === 'plan_generation') {
          // do not redraw graph because plan_generation does not update output_dbs
          var req = that._run_inference("GET");
          req.send();
        } else {
          that._last_module = that._next_module;
          that._next_module = that._get_next_module();
          that.updateGraph(response.result);
          setTimeout( function() {
            that._nextButton.setEnabled(true);
            if (that._last_module == 'senses_and_roles') {
              that._getRoleDist.setEnabled(true); // set enabled when senses_and_roles has finished
            }
          }, response.result.length * 1000);
          if (!that._stepwise) {
            console.log('bumming around for', response.result.length * 1000, ' mseconds before sending new request...');
            setTimeout( function() {
              var req = that._run_inference("GET");
              req.send();
            }, response.result.length * 1000); // wait for graph to be updated
          }
        }
      }, this);
		  return req;
    },

    _get_next_module : function() {
      var moduleReq = new qx.io.request.Xhr(); 
      moduleReq.setUrl("/prac/_pracinfer_get_next_module");
      moduleReq.setMethod('GET');
      moduleReq.setRequestHeader("Cache-Control", "no-cache");
      moduleReq.setRequestHeader("Content-Type", "text/plain");
      moduleReq.addListener("success", function(e) {
        console.log('success');
        var that = this;
        var tar = e.getTarget();
        var response = tar.getResponse();
        console.log('Next module to be executed is:', response);
        that._next_module = response;
        return;
      }, this);
      moduleReq.send();
    },

    buildForm : function()
    {
      var group = new qx.ui.groupbox.GroupBox("Expert Settings");
      var grouplayout = new qx.ui.layout.HBox();
      group.setLayout(grouplayout);
      var formLayout = new qx.ui.layout.Grid();
      formLayout.setRowFlex(0, 1);
      formLayout.setRowFlex(1, 1);
      formLayout.setColumnWidth(2, 70);


      var formgroup = new qx.ui.container.Composite(formLayout);

    /**
     * create form widgets
     */
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

      /**
       * form widget listeners
       */
      module.addListener("changeSelection", this._changeModule, this);

      /**
       * widget settings
       */
      module.add(new qx.ui.form.ListItem("-one-"));
      module.add(new qx.ui.form.ListItem("-two-"));
      logic.add(new qx.ui.form.ListItem("-choose-"));
      kb.add(new qx.ui.form.ListItem("-choose-"));
      mln_dd.add(new qx.ui.form.ListItem("-choose-"));
      evidence_dd.add(new qx.ui.form.ListItem("-choose-"));
      method.add(new qx.ui.form.ListItem("-choose-"));

      /**
       * add form elements to grid
       */
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

    _get_role_distributions : function(e)
    {

      console.log('getting role distributions...');
      var req = new qx.io.request.Xhr(); 
      req.setUrl("/prac/_get_role_distributions");
      req.setMethod('GET');
      req.setRequestHeader("Cache-Control", "no-cache");
      req.setRequestHeader("Content-Type", "application/json");
      var that = this;
      req.addListener("success", function(e) {
        var tar = e.getTarget();
        var response = tar.getResponse();
        if (response.distributions) {
          var left = 20;
          var top = 20;
          for (var role in response.distributions) {
            var distWindow = new qx.ui.window.Window(role);
            distWindow.setWidth(1000);
            distWindow.setHeight(700);
            distWindow.setShowMinimize(false);
            distWindow.setLayout(new qx.ui.layout.Grow());
            var svgCanvas = new qx.ui.embed.Html();
            distWindow.add(svgCanvas);
            this.getRoot().add(distWindow, {left:left, top:top});
            svgCanvas.setHtml(response.distributions[role]);
            distWindow.open();
            left += 20;
            top += 20;
            }      
          return;
        }
      }, that);
      req.send();
    },

    _get_cram_plan : function(e)
    {
      console.log('asking for cram plans...');
      var req = new qx.io.request.Xhr(); 
      req.setUrl("/prac/_get_cram_plan");
      req.setMethod('GET');
      req.setRequestHeader("Cache-Control", "no-cache");
      req.setRequestHeader("Content-Type", "application/json");
      var that = this;
      req.addListener("success", function(e) {
        var tar = e.getTarget();
        var response = tar.getResponse();
        if (response.plans) {
          var cramPlanWindow = new qx.ui.window.Window('Cram Plans');
          cramPlanWindow.setWidth(900);
          cramPlanWindow.setHeight(300);
          cramPlanWindow.setShowMinimize(false);
          cramPlanWindow.setLayout(new qx.ui.layout.Grow());
          var planCanvas = new qx.ui.embed.Html();
          cramPlanWindow.add(planCanvas);
          this.getRoot().add(cramPlanWindow, {left:20, top:20});
          planCanvas.setHtml("<p class='cramPlan'>" + response.plans.join('') + "</p>");
          cramPlanWindow.open();     
          return;
        }
      }, that);
      req.send();
    },

    _load_flow_chart : function(e) {
      console.log('loading flowchart...');
      var req = new qx.io.request.Xhr(); 
      req.setUrl("/prac/_load_flow_chart");
      req.setMethod('GET');
      req.setRequestHeader("Cache-Control", "no-cache");
      req.setRequestHeader("Content-Type", "text/plain");
      var that = this;
      req.addListener("success", function(e) {
        var tar = e.getTarget();
        var response = tar.getResponse();
        this._flowChartEmbed.setHtml(response);             
        return;
      }, that);
      req.send();
    },

    _clearFlowChart : function(e) {
      var nodes = ['init','nl_parsing','ac_recognition','senses_and_roles','executable','plan_generation','achieved_by','roles_transformation'];
      for (var x = 0; x < nodes.length; x++) {
        document.getElementById(nodes[x]).nextElementSibling.style.fill = "white";
      }
    },

    _changeModule : function(e) {
      // request options for other form fiels from server
    },

    /**
     * hide or show expert settings pane
     */
    _changeVisiblity : function(e) {
      if (this._showexpSettingsContainer)
        this._expSettingsContainer.show();
      else 
        this._expSettingsContainer.exclude();
      this._graphVizContainer.show();
    }
  }
});
