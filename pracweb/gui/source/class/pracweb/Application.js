/* ************************************************************************

   Copyright:

   License:

   Authors: Daniel Nyga, Mareike Picklum

************************************************************************ */

/* ************************************************************************


************************************************************************ */

/**
 * This is the main application class of your custom application "pracweb"
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
        var    req = new qx.io.request.Xhr();
        req.setUrl("/prac/_destroy_session");
        req.setMethod("POST");
        req.addListener("success", function(e) { 
            var tar = e.getTarget();                                
            var response = tar.getResponse();
            sessionname = response;
        });
        req.send();
        }; 

      /* ********************** CREATE ELEMENTS ******************************/
      var prac_container = document.getElementById("prac_container", true, true);
      var contentIsle = new qx.ui.root.Inline(prac_container,true,true);
          contentIsle.setWidth(document.getElementById("container", true, true).offsetWidth);
          contentIsle.setHeight(document.getElementById("container", true, true).offsetHeight);
          contentIsle.setLayout(new qx.ui.layout.Grow());

      // scrollable container
      var mainScrollContainer = new qx.ui.container.Scroll().set({
        width: 1024,
        height: 768
      });

      // main container (contains outer splitpane and control panel)
      var container = new qx.ui.container.Composite(new qx.ui.layout.VBox());

      // outer splitpane (contains inference settings and inner splitpane)
      var splitPane = new qx.ui.splitpane.Pane("horizontal");
      this._pane = splitPane;

      // container for inference settings form
      var infSettingsContainer = new qx.ui.container.Composite(new qx.ui.layout.VBox()).set({
        minWidth: .2*document.getElementById("container", true, true).offsetWidth
      });
      this._infSettingsContainer = infSettingsContainer;
      var form = this.buildForm();
      var placeholder = new qx.ui.container.Composite();
      placeholder.setHeight(100);

      // container for visualization elements
      var graphVizContainer = new qx.ui.container.Composite(new qx.ui.layout.Canvas()).set({
        width: .79*document.getElementById("container", true, true).offsetWidth
      });
      this._graphVizContainer = graphVizContainer;

      // embedding for conditional probability png
      var condProb = new qx.ui.basic.Image();
      condProb.setAllowGrowX(true);
      condProb.setAllowShrinkX(true);
      condProb.setAllowGrowY(true);
      condProb.setAllowShrinkY(true);
      condProb.setScale(true);
      this._condProb = condProb;

      var condProbWin = new qx.ui.window.Window("Conditional Probability");
      condProbWin.setWidth(.2*document.getElementById("container", true, true).offsetWidth);
      condProbWin.setHeight(.1*document.getElementById("container", true, true).offsetWidth);
      condProbWin.setShowMinimize(false);
      condProbWin.setLayout(new qx.ui.layout.Canvas());
      condProbWin.setContentPadding(4);
      condProbWin.add(condProb);
      condProbWin.open();
      condProbWin.hide();
      this._condProbWin = condProbWin;

      var waitImage = new qx.ui.basic.Image();
      waitImage.setSource('/prac/static/images/wait.gif');
      waitImage.getContentElement().setAttribute('id', 'waitImg');
      waitImage.setWidth(300);
      waitImage.setHeight(225);
      waitImage.setMarginLeft(-150);
      waitImage.setMarginTop(-125);
      waitImage.setScale(1);
      waitImage.hide();
      this._waitImage = waitImage;

      var vizComposite = new qx.ui.container.Composite()
      vizComposite.setLayout(new qx.ui.layout.Grow());
      vizComposite.getContentElement().setAttribute('id', 'viz');

      var flowChartComposite = new qx.ui.container.Composite()
      flowChartComposite.setLayout(new qx.ui.layout.Grow());
      flowChartComposite.getContentElement().setAttribute('id', 'flowchart');
      this._flowChartComposite = flowChartComposite;

      // control pane (containing text field, buttons and checkboxes used for inference)
      var controlPane = this.buildControlPane();
      this._controlPane = controlPane;

      /* ************************ LISTENERS **********************************/
        prac_container.addEventListener("resize", function() {
            var w = document.getElementById("container", true, true).offsetWidth;
            var h = document.getElementById("container", true, true).offsetHeight;
            contentIsle.setWidth(w);
            contentIsle.setHeight(h);
        }, this);

      document.addEventListener("roll", function(e) {
        this[0].scrollTop = this[0].scrollTop + e.delta.y;
        this[0].scrollLeft = this[0].scrollLeft + e.delta.x;
      }, this);

      window.addEventListener("resize", function() {
        var w = document.getElementById("container", true, true).offsetWidth;
        var h = document.getElementById("container", true, true).offsetHeight;
          contentIsle.setWidth(w);
          contentIsle.setHeight(h);
      }, this);
      // resize image to fit in window
      condProbWin.addListener("resize", function(e) {
        var ratio =  typeof this._imgRatio != 'undefined'? this._imgRatio : 1;
        var newWidth = e.getData().width - 10;
        var newHeight = e.getData().height - 30;
        if (newWidth / ratio <= newHeight) {
          newHeight = newWidth / ratio;
        } else {
          newWidth = newHeight * ratio;
        }
        condProb.setWidth(parseInt(newWidth, 10));
        condProb.setHeight(parseInt(newHeight, 10));
      }, this);

      // resize image to fit in window
      condProb.addListener("changeSource", function(e) {
        var ratio =  typeof this._imgRatio != 'undefined'? this._imgRatio : 1;
        var newWidth = condProbWin.getInnerSize().width - 10;
        var newHeight = condProbWin.getInnerSize().height - 30;
        if (newWidth / ratio <= newHeight) {
          newHeight = newWidth / ratio;
        } else {
          newWidth = newHeight * ratio;
        }
        condProb.setWidth(parseInt(newWidth, 10));
        condProb.setHeight(parseInt(newHeight, 10));
      }, this);


      // reposition graph when inference settings are shown/hidden
      graphVizContainer.addListener('resize', function(e) {
        if (typeof this._graph != 'undefined') {
          var vizSize = graphVizContainer.getInnerSize();
          var bounds = graphVizContainer.getBounds();
          this._graph.w = vizSize.width;
          this._graph.h = vizSize.height;
          this._graph.update();
        }
      }, this);


      /* ********************** SET UP LAYOUT ********************************/
      infSettingsContainer.add(placeholder);
      infSettingsContainer.add(form);
      graphVizContainer.add(flowChartComposite, { left: "80%", top: 0, width: "20%", height:"auto"});
      graphVizContainer.add(vizComposite, { left: 0, top: 0, width: "100%", height:"100%"});
      graphVizContainer.add(waitImage, { left: "50%", top: "50%"});
      splitPane.add(infSettingsContainer, {width: "20%"});
      splitPane.add(graphVizContainer);
      container.add(splitPane, {flex: 1, width: "100%"});
      container.add(controlPane, {width: "100%"});
      mainScrollContainer.add(container);
      contentIsle.add(mainScrollContainer, {width: "100%", height: "100%"});

      /* *************************** INIT ************************************/
      this._load_flow_chart();
      this.getRoot().add(condProbWin, {left:20, top:20});
      this._showinfSettingsContainer = false;
      this._use_exp_settings = false;
      this._stepwise = false;
      this._change_visibility();
      this._send_user_stats();
    },

    /**
     * load initial graph
     */
    loadGraph : function() {
      if (typeof this._graph === 'undefined') {
        this._graph = new pracweb.Graph();
        var vizSize = this._graphVizContainer.getInnerSize();
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
      var win = new qx.ui.window.Window("Wordnet Taxonomy");
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
         getRoleDist.setEnabled(false);
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

      var showFlowchart = new qx.ui.form.CheckBox("Show/hide Flowchart");
      showFlowchart.setValue(true);
      showFlowchart.addListener('changeValue', function(e) {
          var el = this._flowChartComposite.getContentElement().getDomElement();
          if (e.getData())
            this._flowChartComposite.show();
          else
            this._flowChartComposite.hide();
        }, this);

      var showCondProb = new qx.ui.form.CheckBox("Show/hide Cond. Probability");
      showCondProb.addListener('changeValue', function(e) {
          var el = this._condProb.getContentElement().getDomElement();
          if (e.getData())
            this._condProbWin.show();
          else
            this._condProbWin.hide();
        }, this);


        var instructionContainer = new qx.ui.container.Composite(new qx.ui.layout.VBox());
        instructionContainer.add(new qx.ui.basic.Label("Natural-language instruction:"));
        instructionContainer.add(description);

        var buttonContainer = new qx.ui.container.Composite(new qx.ui.layout.VBox());
        buttonContainer.add(vizButton);
        buttonContainer.add(nextButton);

        var buttonContainer2 = new qx.ui.container.Composite(new qx.ui.layout.VBox());
        buttonContainer2.add(wordnetButton);
        buttonContainer2.add(getRoleDist);

        var optionsSlideBar = new qx.ui.container.SlideBar();
        optionsSlideBar.setLayout(new qx.ui.layout.HBox(5));
        optionsSlideBar.add(expSettings);
        optionsSlideBar.add(stepInf);
        optionsSlideBar.add(showFlowchart);
        optionsSlideBar.add(showCondProb);


        mainGroup.add(instructionContainer, {edge: 0, width: "40%"});
        mainGroup.add(buttonContainer, {edge: 0, width: "10%"});
        mainGroup.add(buttonContainer2, {edge: 0, width: "10%"});
        mainGroup.add(optionsSlideBar, {edge: 0, width: "40%"});
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
        this._imgRatio = response.ratio;
        this._condProb.resetSource();
        if (response.img !== '') {
          this._condProb.setSource('data:image/png;base64,' + response.img);
        }
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
      formLayout.setColumnWidth(0, 130); // col 0 has width 130
      formLayout.setColumnWidth(2, 270); // col 2 has width 270
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

      var mlnAreaContainerLayout = new qx.ui.layout.Grow();
      var mlnAreaContainer = new qx.ui.container.Composite(mlnAreaContainerLayout);
      var textAreaMLN = new qx.ui.form.TextArea("");
      this.__textAreaMLN = textAreaMLN;
      textAreaMLN.setMinWidth(300);
      textAreaMLN.getContentElement().setAttribute("id", 'mlnArea');
      textAreaMLN.addListener("appear", function() {
                this._highlight(textAreaMLN.getContentElement().getAttribute('id'));
            }, this);
      mlnAreaContainer.add(this.__textAreaMLN);

      var evidenceLabel = new qx.ui.basic.Label().set({
        value: this._template('Evidence:', 'label'),
        rich: true
      });

      var evidenceContainerLayout = new qx.ui.layout.Grow();
      var evidenceContainer = new qx.ui.container.Composite(evidenceContainerLayout);
      var textAreaEvidence = new qx.ui.form.TextArea("");
      this.__textAreaEvidence = textAreaEvidence;
      textAreaEvidence.setMinWidth(300);
      textAreaEvidence.getContentElement().setAttribute("id", 'evidenceArea');
      textAreaEvidence.addListener("appear", function() {
                this._highlight(textAreaEvidence.getContentElement().getAttribute('id'));
            }, this);
      evidenceContainer.add(textAreaEvidence);


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

      group.add(formgroup, {height: "20%"});

      group.add(mlnLabel, {height: "5%"});
      group.add(mlnAreaContainer, {height: "35%"});

      group.add(evidenceLabel, {height: "5%"});
      group.add(evidenceContainer, {height: "35%"});

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
        this._flowChartComposite.getContentElement().getDomElement().innerHTML = response;
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

      var mlnContent = typeof settings['mln'] === 'undefined' ? '' : settings['mln'];
      this.__textAreaMLN.setValue(mlnContent);
      this._highlight(this.__textAreaMLN.getContentElement().getAttribute('id'));

      this.__textAreaEvidence.setValue(this._oldEvidence);
      this._highlight(this.__textAreaEvidence.getContentElement().getAttribute('id'));
    },

    /**
     * show or hide animated wait logo
     */
    _show_wait_animation : function(wait) {
      if (wait){
        this._waitImage.show();
      } else {
        this._waitImage.hide();
      }
    },

    /**
     * formatting template for inf settings labels and text
     */
    _template : function(val, type) {
        if (type === 'label')
            return '<span style="font-size:13px; font-weight:bold">' + val + '</span>'
        else
            return '<b>' + val + '</b>';
    },

    /**
     * send user statistics to server
     */
    _send_user_stats : function() {
        var currentdate = new Date();
        var date = currentdate.getDate() + "/"
                + (currentdate.getMonth()+1)  + "/"
                + currentdate.getFullYear();
        var time = currentdate.getHours() + ":"
                + currentdate.getMinutes() + ":"
                + currentdate.getSeconds();
//        var url = 'http://jsonip.appspot.com?callback=?';
        var url = 'https://api.ipify.org?format=jsonp';
        var req = new qx.bom.request.Jsonp();

        var reqServer = new qx.io.request.Xhr();
            reqServer.setUrl("/prac/_user_stats");
            reqServer.setMethod("POST");
            reqServer.setRequestHeader("Cache-Control", "no-cache");
            reqServer.setRequestHeader("Content-Type", "application/json");

        req.onload = function() {
          reqServer.setRequestData({ 'ip': req.responseJson.ip, 'date': date, "time": time });
          reqServer.send();
        }
        req.onerror = function() {
          reqServer.setRequestData({ 'ip': null, 'date': date, "time": time });
          reqServer.send();
        }
        req.open("GET", url);
        req.send();
    },

    /**
    * Syntax highlighting
    */
    _highlight : function(id) {
        if (document.getElementById(id)) {
            var code = CodeMirror.fromTextArea(document.getElementById(id), {
                lineNumbers: true
            });

            // save codemirror to be able to get the content later
            this['codeMirror' + id] = code;
        }
    },

    /**
     * hide or show expert settings pane
     */
    _change_visibility : function(e) {

      if (this._showinfSettingsContainer) {
        this._infSettingsContainer.show();
      } else
        this._infSettingsContainer.exclude();
        this._graphVizContainer.show();
    }
  }
});
