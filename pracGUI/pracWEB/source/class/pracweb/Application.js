/* ************************************************************************

   Copyright:

   License:

   Authors:

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

      // Enable logging in debug variant
      if (qx.core.Environment.get("qx.debug"))
      {
        // support native logging capabilities, e.g. Firebug for Firefox
        qx.log.appender.Native;
        // support additional cross-browser console. Press F7 to toggle visibility
        qx.log.appender.Console;
      }

      var contentIsle = new qx.ui.root.Inline(document.getElementById("container", true, true));

      // new container
      var container = new qx.ui.container.Composite(new qx.ui.layout.VBox(0)).set({
        padding: 0,
        allowShrinkX: false,
        allowShrinkY: false
      });

      var splitPane = new qx.ui.splitpane.Pane("horizontal");
      splitPane.setHeight(400);

      this.__pane = splitPane;

      // Create container with fixed dimensions for the left:
      var left = new qx.ui.container.Composite(new qx.ui.layout.Grow).set({
        width : 200,
        height: 100,
        decorator : "main"
      });

      // Create container for the right:
      var right = new qx.ui.container.Composite(new qx.ui.layout.VBox(10)).set({
        padding : 10,
        maxWidth : 450,
        decorator : "main"
      });


      // Left
      var form = this.buildForm();
      left.add(form);

      // Right
      var vizEmbedGrp = new qx.ui.groupbox.GroupBox("Visualization", "icon/16/apps/utilities-text-editor.png");

      var vizLayout = new qx.ui.layout.HBox(0);
      vizEmbedGrp.setLayout(vizLayout);

      var vizHTML = "<div id='viz'></div>";
      var vizEmbed = new qx.ui.embed.Html(vizHTML);
      vizEmbedGrp.add(vizEmbedGrp);
      var playButton = new qx.ui.form.Button("Play");
      playButton.addListener();
      var fwdButton = new qx.ui.form.Button("Next");
      var clearButton = new qx.ui.form.Button("Clear");


      right.add(vizEmbedGrp);
      right.add(playButton);
      right.add(fwdButton);
      right.add(clearButton);

      this._left = left;
      this._right = right;

      splitPane.add(left, 0);
      splitPane.add(right, 1);

      var mainPane = this.buildMainPane();
      this._mainPane = mainPane;

      container.add(splitPane, { height : "85%" });
      container.add(mainPane);

      // add container to content div
      contentIsle.add(container);
    },

    buildMainPane : function()
    {
      var mainGroup = new qx.ui.groupbox.GroupBox("PRAC Inference", "icon/16/apps/utilities-text-editor.png");

      var mainLayout = new qx.ui.layout.HBox(20);
      mainGroup.setLayout(mainLayout);
      
      var description = new qx.ui.form.TextField("");
      var expSettings = new qx.ui.form.CheckBox("Use expert settings");
      expSettings.addListener("changeValue", this._changeVisiblity, this);
      expSettings.setValue(true);
      expSettings.setValue(false);
      var stepInf = new qx.ui.form.CheckBox("Step-by-step inference");
      var vizButton = new qx.ui.form.Button("Play");

      mainGroup.add(new qx.ui.basic.Label("Description"));
      mainGroup.add(description);
      mainGroup.add(expSettings);
      mainGroup.add(stepInf);
      mainGroup.add(vizButton);

      return mainGroup;
    },

    buildForm : function()
    {
      // build form
      var group = new qx.ui.groupbox.GroupBox("Expert Settings", "icon/16/apps/utilities-text-editor.png");
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
      module.add(new qx.ui.form.ListItem("-choose-"));
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
      formgroup.add(mlnGroup, {row: 3, column: 2});
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

    _changeModule : function(e)
    {
      // alert("send..." + e);
    },


    _changeVisiblity : function(e)
    {
      if(e.getData())
      {
        this._left.show();
        this._right.show();
      }
      else
      {
        this._left.exclude();
        this._right.show();
      }
    }
  }
});
