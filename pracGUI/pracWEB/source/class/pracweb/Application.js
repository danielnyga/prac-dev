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
      dp.SyntaxHighlighter.ClipboardSwf = "/prac/static/script/clipboard.swf') }}";
	  
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
			sessionname = response;
		});
		req.send();
  	  }; 

      // main frame
      var contentIsle = new qx.ui.root.Inline(document.getElementById("prac_container", true, true));
      contentIsle.setWidth(document.getElementById("container", true, true).offsetWidth);
      contentIsle.setHeight(document.getElementById("container", true, true).offsetHeight);
      contentIsle.setLayout(new qx.ui.layout.Grow());
      window.addEventListener("resize", function() {
        var w = document.getElementById("container", true, true).offsetWidth;
        var h = document.getElementById("container", true, true).offsetHeight;
      	contentIsle.setWidth(w);
      	contentIsle.setHeight(h);
      }, this);

      // scrollable container 
      var mainScrollContainer = new qx.ui.container.Scroll().set({
        width: 1024,
        height: 768
      });
	
      // main container (contains outer splitpane and control panel)
      var container = new qx.ui.container.Composite(new qx.ui.layout.VBox()).set({
        padding: 0
      });

      mainScrollContainer.add(container);

      // outer splitpane (contains inference settings and inner splitpane)
      var splitPane = new qx.ui.splitpane.Pane("horizontal");
      this._pane = splitPane;

      // container for inference settings form
      var infSettingsContainer = new qx.ui.container.Composite(new qx.ui.layout.VBox()).set({
        minWidth: 370
      });
      this._infSettingsContainer = infSettingsContainer;


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
        height: 500
      });

      var flowChartContainer = new qx.ui.container.Composite(new qx.ui.layout.VBox()).set({
        width: 444,
        minWidth: 444,
        height: 400,
        minHeight: 503
       });

      flowChartScroll.add(flowChartContainer);
      this._flowChartScroll = flowChartScroll;

      // embedding for flowchart svg
      var flowChartEmbed = new qx.ui.embed.Html();
      flowChartEmbed.setMinHeight(503);
      this._flowChartEmbed = flowChartEmbed;
      flowChartContainer.add(flowChartEmbed, {flex: 1});
      this._load_flow_chart();

      // embedding for conditional probability png
      var condProb = new qx.ui.basic.Image();
      this._condProb = condProb;
      condProb.setScale(true);
      condProb.setMarginTop(20);
      condProb.setMinHeight(300);
      flowChartContainer.add(condProb, { top:0 });


      // resize conditional probability png with flowchart container
      flowChartScroll.addListener("resize", function(e) {
        condProb.setWidth(e.getData().width); 
        condProb.setHeight(e.getData().width / this._imgRatio);
      });

      // resize flowchartcontainer to create scrollbars if the cond prob png is too large
      condProb.addListener("changeSource", function(e) {
        flowChartContainer.setMinHeight(600 + this._flowChartScroll.getInnerSize().width  / this._imgRatio);
      }, this);

      // expert settings
      var form = this.buildForm();

      // placeholder
      var placeholder = new qx.ui.container.Composite();
      placeholder.setHeight(100);
      infSettingsContainer.add(placeholder);
      infSettingsContainer.add(form);

      // visualization of svg graph
      var vizEmbedGrp = new qx.ui.container.Composite();;
      var vizLayout = new qx.ui.layout.Grow();
      vizEmbedGrp.setLayout(vizLayout);
      var vizHTML = "<div id='viz'></div>";
      var waitHTML = "<img id='waitImg' src='/prac/static/images/wait.gif'>";
      var waitEmbed = new qx.ui.embed.Html(waitHTML);
      waitEmbed.hide();
      this._waitEmbed = waitEmbed;
      var vizEmbed = new qx.ui.embed.Html(vizHTML);
      vizEmbedGrp.add(vizEmbed);
      vizEmbedGrp.add(waitEmbed);
      this._vizEmbedGrp = vizEmbedGrp;

      // reposition graph when inference settings are shown/hidden
      vizEmbedGrp.addListener('resize', function(e) {
        if (typeof this._graph != 'undefined') {
          var vizSize = vizEmbedGrp.getInnerSize();
          var bounds = vizEmbedGrp.getBounds();
          this._graph.w = vizSize.width;
          this._graph.h = vizSize.height;
          this._graph.update();
        }
      }, this);

      graphVizContainer.add(vizEmbedGrp);
      this._flowChartContainer = flowChartContainer;

      innerSplitPane.add(graphVizContainer);
      innerSplitPane.add(flowChartScroll);

      splitPane.add(infSettingsContainer, 0);
      splitPane.add(innerSplitPane, 1);

      // control pane (containing text field, buttons and checkboxes used for inference)
      var controlPane = this.buildControlPane();
      this._controlPane = controlPane;

      container.add(splitPane, {flex: 1});
      container.add(controlPane);

      // add container to content div
      contentIsle.add(mainScrollContainer);
      
      // initially do not show form and do NOT use stepwise inference by default
      this._showinfSettingsContainer = false;
      this._use_exp_settings = false;
      this._stepwise = false;
      this._change_visibility();
    },

    /**
     * load initial graph
     */
    loadGraph : function() {
      if (typeof this._graph === 'undefined') {
        this._graph = new pracweb.Graph();
        var vizSize = this._vizEmbedGrp.getInnerSize();
        this._graph.w = vizSize.width;
        this._graph.h = vizSize.height;
      } 
      this._graph.clear();
    },

    /**
     * trigger redrawing of graph without animation;
     * graph will replaced immediately
     */
    redrawGraph : function(data) {
      this._graph.replaceData(data);
    },

    /**
     * trigger redrawing of graph with bouncy visualization
     */
    updateGraph : function(removeLinks, addLinks) {
      this._graph.updateData(removeLinks, addLinks);
    },

    /**
     * set up layout of control pane including instruction field and buttons
     */
    buildControlPane : function()
    {
      var mainGroup = new qx.ui.groupbox.GroupBox("PRAC Inference");
      var mainLayout = new qx.ui.layout.HBox(20);
      mainGroup.setLayout(mainLayout);
      
      // combobox containing nl descriptions
      var description = new qx.ui.form.ComboBox();
      description.setWidth(500);
      description.add(new qx.ui.form.ListItem("start the centrifuge."));
      description.add(new qx.ui.form.ListItem("Add some water to the purine."));
      description.add(new qx.ui.form.ListItem("Add some arsenic_acid to the imidazole."));
      description.add(new qx.ui.form.ListItem("Add 5 drops of the lysergic_acid to the pyrimidine."));
      description.add(new qx.ui.form.ListItem("Add 1 liter of water to the chlorous_acid."));
      description.add(new qx.ui.form.ListItem("Neutralize the methacrylic_acid with 100 milliliters of cyanuramide."));
      description.add(new qx.ui.form.ListItem("start with neutralizing the pyridine with 4 drops of hydrofluoric_acid."));
      description.add(new qx.ui.form.ListItem("Open the drawer."));
      description.add(new qx.ui.form.ListItem("Open the fridge."));
      description.add(new qx.ui.form.ListItem("Open the test_tube."));
      description.add(new qx.ui.form.ListItem("Open the cupboard."));
      description.setValue(description.getChildrenContainer().getSelectables()[0].getLabel());
      
      description.addListener("keydown", function(e) {
        this._vizButton.setEnabled(true);
        this._clear_flow_chart();
        document.getElementById('init').nextElementSibling.style.fill = "#bee280";
      }, this);

      description.addListener("changeValue", function(e) {
        this._vizButton.setEnabled(true);
        this._clear_flow_chart();
        document.getElementById('init').nextElementSibling.style.fill = "#bee280";
      }, this);
      
      var expSettings = new qx.ui.form.CheckBox("Show Inference settings");
      expSettings.addListener("changeValue", function(e) {
        this._showinfSettingsContainer = e.getData();
        this._change_visibility();
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
          var tar = e.getTarget();                
          var response = tar.getResponse();
          taxCanvas.setHtml(response);
          win.open();
        });
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
         this._clear_flow_chart();
         this._next_module = 'nl_parsing';
         document.getElementById('init').nextElementSibling.style.fill = "#bee280";
         this._oldRes = {};
         this._oldEvidence = description.getValue();
         var req = this._run_inference("POST");
         req.setRequestHeader("Content-Type", "application/json");
         req.setRequestData({ 'sentence': description.getValue() });
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
      mainGroup.add(expSettings); // temporarily removed for openEase integration
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
      this._show_wait_animation(true);
      this._nextButton.setEnabled(false);

      // update flowchart
      if (this._next_module === 'achieved_by' || (this._next_module === 'plan_generation') && (this._last_module != 'plan_generation')) {
        var that = this;
        var tmpNM = that._next_module; //because following timeout will cause _next_module to be overwritten too quickly
        that._clear_flow_chart();
        document.getElementById('executable').nextElementSibling.style.fill = "#bee280";
        setTimeout( function() {
          that._clear_flow_chart();
          document.getElementById(tmpNM).nextElementSibling.style.fill = "#bee280";
        }, 1000);
      } else {
        this._clear_flow_chart();
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

        var responseResult = response.result;
        var responseSettings = response.settings == null ? {} : response.settings;
        that._set_exp_settings(responseSettings);


        // determine links to be removed/added
        var updateLinks = that._calculateRedrawing(that._oldRes, responseResult);
        var idle_time = 1000 + (updateLinks[0].length + updateLinks[1].length) * 500;
        that._oldRes = responseResult;
        that._oldEvidence = responseSettings == null ? '' : responseSettings['evidence'];

        that._show_wait_animation(false);
        if (response.finish) {
          console.log(" I am DONE! ");
          that._show_wait_animation(true);
          that.updateGraph(updateLinks[0], updateLinks[1]);
          setTimeout( function() {
            that._get_cram_plan();
          }, idle_time); // wait 3 seconds, then clear flowchart
          that._nextButton.setEnabled(false);
          that._vizButton.setEnabled(true);
          that._last_module = '';
          that._show_wait_animation(false);
        } else if (that._next_module === 'plan_generation') {
          // do not redraw graph because plan_generation does not update output_dbs
          var req = that._run_inference("GET");
          req.send();
        } else {
          that._last_module = that._next_module;
          that._next_module = that._get_next_module();
          that._get_cond_prob();
          that.updateGraph(updateLinks[0], updateLinks[1]);
          setTimeout( function() {
            that._nextButton.setEnabled(true);
            if (that._last_module == 'senses_and_roles') {
              that._getRoleDist.setEnabled(true); // set enabled when senses_and_roles has finished
            }
          }, idle_time);
          if (!that._stepwise) {
            console.log('bumming around for', idle_time, ' mseconds before sending new request...');
            setTimeout( function() {
              var req = that._run_inference("GET");
              req.send();
            }, idle_time); // wait for graph to be updated
          }
        }
      }, this);
		  return req;
    },

    /**
     * get name of the next module to be executed
     */
    _get_next_module : function() {
      var moduleReq = new qx.io.request.Xhr(); 
      moduleReq.setUrl("/prac/_pracinfer_get_next_module");
      moduleReq.setMethod('GET');
      moduleReq.setRequestHeader("Cache-Control", "no-cache");
      moduleReq.setRequestHeader("Content-Type", "application/json");
      moduleReq.addListener("success", function(e) {
        var that = this;
        var tar = e.getTarget();
        var response = tar.getResponse();
        that._next_module = response;
        return;
      }, this);
      moduleReq.send();
    },

    /**
     * get the names of all pracmodules
     * --currently not in use--
     */
    _get_modules : function() {
      var moduleReq = new qx.io.request.Xhr(); 
      moduleReq.setUrl("/prac/_get_modules");
      moduleReq.setMethod('GET');
      moduleReq.setRequestHeader("Cache-Control", "no-cache");
      moduleReq.setRequestHeader("Content-Type", "application/json");
      moduleReq.addListener("success", function(e) {
        var that = this;
        var tar = e.getTarget();
        var response = tar.getResponse();
        for (var i = 0; i < response.modules.length; i++) {
          this.moduleSelect.add(new qx.ui.form.ListItem(response.modules[i]));
        }
        for (var i = 0; i < response.methods.length; i++) {
          this.methodSelect.add(new qx.ui.form.ListItem(response.methods[i][1], null, response.methods[i][0]));
        }

      }, this);
      moduleReq.send();
    },

    /**
     * request conditional probability png for current inference step
     * and display it under the flowchart
     */
    _get_cond_prob : function() {
      var moduleReq = new qx.io.request.Xhr(); 
      moduleReq.setUrl("/prac/_pracinfer_get_cond_prob");
      moduleReq.setMethod('GET');
      moduleReq.setRequestHeader("Cache-Control", "no-cache");
      moduleReq.setRequestHeader("Content-Type", "application/json");
      moduleReq.addListener("success", function(e) {
        var that = this;
        var tar = e.getTarget();
        var response = tar.getResponse();
        this._condProb.resetSource();
        if (response.img !== '') {
          this._condProb.setSource('data:image/png;base64,' + response.img);
        }
        this._imgRatio = response.ratio;
        return;
      }, this);
      moduleReq.send();
    },    

    /**
     * calculate the links to be added and the links
     * to be removed from the visualization graph
     */
    _calculateRedrawing : function(oldRes, newRes) {
      var toBeRemoved = [];
      var toBeAdded = [];
      var remove;
      var add;

      // old links to be removed
      for (var i = 0; i < oldRes.length; i++) {
        remove = true;
        for (var j = 0; j < newRes.length; j++) {
          // if there is already a link between the nodes, do not remove it
          if (oldRes[i].source === newRes[j].source && oldRes[i].target === newRes[j].target && oldRes[i].value === newRes[j].value) {
            remove = false;
            break;
          }
        }
        if (remove) {
          toBeRemoved.push(oldRes[i]);
        }
      }

      // new links to be added
      for (var i = 0; i < newRes.length; i++) {
        add = true;
        for (var j = 0; j < oldRes.length; j++) {
          // if there is already a link, do not add it
          if (newRes[i].target === oldRes[j].target && newRes[i].source === oldRes[j].source && newRes[i].value === oldRes[j].value) {
            add = false;
            break;
          }
        }
        if (add) {
          toBeAdded.push(newRes[i]);
        }
      }

      return [toBeRemoved, toBeAdded];
    },

    /**
     * calculate number of steps it will take to redraw the graph
     * will be used to calculate the wait time before sending new request to server
     */
    _diffResults : function(oldRes, newRes) {
      var redrawSteps = 0;
      var redraw;

      // number of old elements to be removed
      for (var i = 0; i < oldRes.length; i++) {
        redraw = true;
        for (var j = 0; j < newRes.length; j++) {
          if (oldRes[i].target === newRes[j].target 
            && oldRes[i].source === newRes[j].source
            && oldRes[i].value === newRes[j].value) {
            redraw = false;
            break;
          }
        }
        redrawSteps += redraw ? 0 : 1;
      }

      // number of new elements to be drawn
      for (var i = 0; i < newRes.length; i++) {
        redraw = true;
        for (var j = 0; j < oldRes.length; j++) {
          if (newRes[i].target === oldRes[j].target 
            && newRes[i].source === oldRes[j].source
            && newRes[i].value === oldRes[j].value) {
            redraw = false;
            break;
          }
        }
        redrawSteps += redraw ? 1 : 0;
      }
      return redrawSteps;
    },

    /**
     * build inference settings pane
     */
    buildForm : function()
    {
      var grouplayout = new qx.ui.layout.VBox();
      var group = new qx.ui.container.Composite(grouplayout);

      var formLayout = new qx.ui.layout.Grid();
      formLayout.setColumnWidth(0, 130); // col 0 has width 150
      formLayout.setColumnWidth(2, 270); // col 0 has width 150
      formLayout.setColumnAlign(0, "left", "middle");
      formLayout.setColumnAlign(1, "left", "middle");
      var formgroup = new qx.ui.container.Composite(formLayout);

    /**
     * create form widgets
     */
      var moduleLabel = new qx.ui.basic.Label().set({
        value: this._template('Module:', 'label'),
        rich : true
      });
      var moduleSelect = new qx.ui.basic.Label().set({
        rich : true
      });
      this.moduleSelect = moduleSelect;

      var logicLabel = new qx.ui.basic.Label().set({
        value: this._template('Logic:', 'label'),
        rich : true
      });
      var logic = new qx.ui.basic.Label().set({
        rich : true
      });
      this.logicSelect = logic;

      var methodLabel = new qx.ui.basic.Label().set({
        value: this._template('Method:', 'label'),
        rich : true
      });
      var method = new qx.ui.basic.Label().set({
        rich : true
      });
      this.methodSelect = method;

      var queriesLabel = new qx.ui.basic.Label().set({
        value: this._template('Queries:', 'label'),
        rich : true
      });
      var queries = new qx.ui.basic.Label().set({
        rich : true
      });
      this.queriesField = queries;

      var paramsLabel = new qx.ui.basic.Label().set({
        value: this._template('Parameters:', 'label'),
        rich : true
      });
      var parameters = new qx.ui.basic.Label().set({
        rich : true
      });
      this.parametersField = parameters;

      var cwPredsLabel = new qx.ui.basic.Label().set({
        value: this._template('CW Preds:', 'label'),
        rich : true
      });
      var cwPreds = new qx.ui.basic.Label().set({
        rich : true
      });
      this.cwPredsField = cwPreds;

      var cwAssLabel = new qx.ui.basic.Label().set({
        value: this._template('CW Assumption:', 'label'),
        rich : true
      });
      var closedWorld = new qx.ui.basic.Label().set({
        rich : true
      });
      this.closedWorld = closedWorld;

      var mlnLabel = new qx.ui.basic.Label().set({
        value: this._template('MLN:', 'label'),
        rich : true
      });
      var mlnField = new qx.ui.embed.Html();
      mlnField.setMinWidth(300);
      mlnField.setMinHeight(320);
      this.mlnField = mlnField;

      var evidenceLabel = new qx.ui.basic.Label().set({
        value: this._template('Evidence:', 'label'),
        rich: true
      });
      var evidenceField = new qx.ui.embed.Html();
      evidenceField.setMinWidth(300);
      evidenceField.setMinHeight(320);
      this.evidenceField = evidenceField;

      /**
       * arrange form elements in grid
       */
      formgroup.add(moduleLabel, {row: 0, column: 0});
      formgroup.add(moduleSelect, {row: 0, column: 1});

      formgroup.add(logicLabel, {row: 1, column: 0});
      formgroup.add(logic, {row: 1, column: 1});

      formgroup.add(methodLabel, {row: 2, column: 0});
      formgroup.add(method, {row: 2, column: 1});

      formgroup.add(queriesLabel, {row: 3, column: 0});
      formgroup.add(queries, {row: 3, column: 1});

      formgroup.add(cwPredsLabel, {row: 4, column: 0});
      formgroup.add(cwPreds, {row: 4, column: 1});

      formgroup.add(cwAssLabel, {row: 5, column: 0});
      formgroup.add(closedWorld, {row: 5, column: 1});

      group.add(formgroup);

      group.add(mlnLabel);
      group.add(mlnField);

      group.add(evidenceLabel);
      group.add(evidenceField);

      return group;
    },

    /**
     * load the role distribution svgs for each role and
     * open them in separate windows
     */
    _get_role_distributions : function(e)
    {

      console.log('getting role distributions...');
      this._show_wait_animation(true);
      var req = new qx.io.request.Xhr(); 
      req.setUrl("/prac/_get_role_distributions");
      req.setMethod('GET');
      req.setRequestHeader("Cache-Control", "no-cache");
      req.setRequestHeader("Content-Type", "application/json");
      var that = this;
      req.addListener("success", function(e) {
        this._show_wait_animation(false);
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

    /**
     * request cram plan for instruction and open it in new window
     */
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
          var planField = new qx.ui.form.TextArea("").set({
            font: qx.bom.Font.fromString("14px monospace")
          });
          cramPlanWindow.add(planField);
          this.getRoot().add(cramPlanWindow, {left:20, top:20});
          planField.setValue(response.plans.join(''));
          cramPlanWindow.open();     
          return;
        }
      }, that);
      req.send();
    },


    /**
     * load flowchart svg into embedding
     */
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

    /**
     * clear flow chart
     */
    _clear_flow_chart : function(e) {
      var nodes = ['init','nl_parsing','ac_recognition','senses_and_roles','executable','plan_generation','achieved_by','roles_transformation'];
      for (var x = 0; x < nodes.length; x++) {
        document.getElementById(nodes[x]).nextElementSibling.style.fill = "white";
      }
    },


    // _highlight_code : function(scriptname) {
    //   console.log('highlight code');
    //   dp.SyntaxHighlighter.ClipboardSwf = "/prac/static/script/clipboard.swf') }}";
    //   dp.SyntaxHighlighter.HighlightAll('code',true);
    // },

    // _change_module : function(e) {
    //   console.log('changing module, requesting selection options');
    //   var req = new qx.io.request.Xhr(); 
    //   req.setUrl("/prac/update_module");
    //   req.setMethod('POST');
    //   req.setRequestHeader("Cache-Control", "no-cache");
    //   req.setRequestHeader("Content-Type", 'application/json');
    //   console.log('sending module', this.moduleSelect.getSelection()[0].getLabel() );
    //   req.setRequestData({ "module": this.moduleSelect.getSelection()[0].getLabel() });
    //   var that = this;
    //   req.addListener("success", function(e) {
    //     var tar = e.getTarget();
    //     var response = tar.getResponse();
    //     console.log(response);
    //     this._update_selections(response);
    //     return;
    //   }, that);
    //   req.send();
    // },

    /**
     * update values for inference settings of current step
     */
    _set_exp_settings : function(settings) {
      typeof settings['module'] != 'undefined' ? this.moduleSelect.setValue(this._template(settings['module'])) : this.moduleSelect.resetValue();
      typeof settings['logic'] != 'undefined' ? this.logicSelect.setValue(this._template(settings['logic'])) : this.logicSelect.resetValue();
      typeof settings['method'] != 'undefined' ? this.methodSelect.setValue(this._template(settings['method'])) : this.methodSelect.resetValue();
      typeof settings['queries'] != 'undefined' ? this.queriesField.setValue(this._template(settings['queries'])) : this.queriesField.resetValue();
      typeof settings['cwPreds'] != 'undefined' ? this.cwPredsField.setValue(this._template(settings['cwPreds'])) : this.cwPredsField.resetValue();
      typeof settings['closedWorld'] != 'undefined' ? this.closedWorld.setValue(this._template(settings['closedWorld'] ? 'true' : 'false')) : this.closedWorld.resetValue();

      var tmpEvidenceHTML = '<textarea name="code" class="mln">' + this._oldEvidence + '</textarea>';
      var evidenceHTML = dp.SyntaxHighlighter.HighlightGivenHTML('code', tmpEvidenceHTML, true, false, false, 1, false);
      this.evidenceField.setHtml(evidenceHTML);

      var mlnContent;
      if (typeof settings['mln'] === 'undefined') {
        mlnContent = '';
      } else {
        mlnContent = settings['mln'];
      }
      var tmpMLNHTML = '<textarea name="code" class="mln">' + mlnContent + '</textarea>';  
      var mlnHTML = dp.SyntaxHighlighter.HighlightGivenHTML('code', tmpMLNHTML, true, false, false, 1, false);
      this.mlnField.setHtml(mlnHTML);
    },

    /**
     * show or hide animated wait logo
     */
    _show_wait_animation : function(wait) {
      if (wait){
        this._waitEmbed.show();
      } else {
         console.log('resetting source');
        this._waitEmbed.hide();
      }
    },



    // _update_mln_text : function(e) {
    //   if (e.getData().length > 0) {
    //     var selection = e.getData()[0].getLabel();
    //     this._update_text(selection, this.mlnArea);
    //   } else {
    //     this.mlnArea.setValue('');
    //   }
    // },

    // _update_evidence_text : function(e) {
    //   if (e.getData().length > 0) {
    //     var selection = e.getData()[0].getLabel();
    //     this._update_text(selection, this.evidenceArea);
    //   } else {
    //     this.evidenceArea.setValue('');
    //   }
    // },

    // _update_text : function(selection, area) {
    //   var that = this;
    //   console.log('changing text, requesting...');
    //   var module = this.moduleSelect.getSelection()[0].getLabel();
    //   var req = new qx.io.request.Xhr(); 
    //   req.setUrl("/prac/update_text");
    //   req.setMethod('POST');
    //   req.setRequestHeader("Cache-Control", "no-cache");
    //   req.setRequestHeader("Content-Type", 'application/json');
    //   req.setRequestData({ "module": module , "fName": selection });
    //   req.addListener("success", function(e) {
    //     var tar = e.getTarget();
    //     var response = tar.getResponse();
    //     area.setValue(response.text);

    //     return;
    //   }, that);
    //   req.send();
    //   // request options for other form fields from server
    // },

    /**
     * update selection items in expert settings
     */
    // _update_selections : function(data) {
    //   // update kb selections
    //   this.kbSelect.removeAll();
    //   for (var k = 0; k < data.kblist.length; k++) {
    //     this.kbSelect.add(new qx.ui.form.ListItem(data.kblist[k]));
    //   }

    //   // update mln selections
    //   this.mlnSelect.removeAll();
    //   console.log('mlnlist', data.mlnlist);
    //   for (var m = 0; m < data.mlnlist.length; m++) {
    //     console.log('adding', data.mlnlist[m]);
    //     this.mlnSelect.add(new qx.ui.form.ListItem(data.mlnlist[m]));
    //   }

    //   // update evidence selections
    //   this.evidenceSelect.removeAll();
    //   for (var e = 0; e < data.evidencelist.length; e++) {
    //     this.evidenceSelect.add(new qx.ui.form.ListItem(data.evidencelist[e]));
    //   }
    // },

    /**
     * formatting template for inf settings labels and text
     */
    _template : function(val, type) {
        if (type === 'label')
            return '<span style="font-size:15px; font-weight:bold">' + val + '</span>'
        else
            return '<b>' + val + '</b>';
    },

    /**
     * hide or show expert settings pane
     */
    _change_visibility : function(e) {

      if (this._showinfSettingsContainer)
        this._infSettingsContainer.show();
      else
        this._infSettingsContainer.exclude();
        this._graphVizContainer.show();
    }
  }
});
