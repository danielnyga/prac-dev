/* ************************************************************************
   Copyright:
   License:
   Authors: Mareike Picklum
************************************************************************ */

/**
 * This is the main application class of your custom application "pracweb"
 *
 * @asset(pracweb/*)
 */
qx.Class.define("pracweb.Application", {
    extend : qx.application.Inline,


/*
*****************************************************************************
    MEMBERS
*****************************************************************************
*/

members : {

    /**
     * This method contains the initial application code and gets called
     * during startup of the application
     *
     * @lint ignoreDeprecated(alert)
     */

    main : function() {

        // Call super class
        this.base(arguments);
        var that = this;

        // Enable logging in debug variant
        if (qx.core.Environment.get("qx.debug")) {
            // support native logging capabilities, e.g. Firebug for Firefox
            qx.log.appender.Native;
            // support additional cross-browser console. Press F7 to toggle
            // visibility
            qx.log.appender.Console;
        }

        // destroy the session before leaving PRAC
        window.onbeforeunload = function () {
            var req = new qx.io.request.Xhr();
            req.setUrl("/prac/_destroy_session");
            req.setMethod("POST");
            req.addListener("success", function(e) {
                var tar = e.getTarget();
                var response = tar.getResponse();
                var sessionname = response;
            });
            req.send();
        };

        /* **************************** COMMON *******************************/

        /* ********************** CREATE ELEMENTS ****************************/

        var ctr_prac = document.getElementById("prac_container",
                                                     true,
                                                     true);
        var inline_content = new qx.ui.root.Inline(ctr_prac,true,true);
        inline_content.setWidth(document.getElementById("page", true, true)
                                   .offsetWidth);
        inline_content.setHeight(document.getElementById("page", true, true)
                                    .offsetHeight);
        inline_content.setLayout(new qx.ui.layout.Grow());
        inline_content.setNativeContextMenu(true);

        // scrollable container
        var ctr_mainscroll = new qx.ui.container.Scroll().set({
            width: 1024,
            height: 768
        });

        // main layout container
        var ctr_main = new qx.ui.container.Composite(new qx.ui.layout.Canvas);

        /* ************************ LISTENERS ********************************/

        ctr_prac.addEventListener("resize", function() {
            var w = document.getElementById("page", true, true).offsetWidth;
            var h = document.getElementById("page", true, true).offsetHeight;
            inline_content.setWidth(w);
            inline_content.setHeight(h);
        }, this);

        document.addEventListener("roll", function(e) {
            this[0].scrollTop = this[0].scrollTop + e.delta.y;
            this[0].scrollLeft = this[0].scrollLeft + e.delta.x;
        }, this);

        window.addEventListener("resize", function() {
            var w = document.getElementById("page", true, true).offsetWidth;
            var h = document.getElementById("page", true, true).offsetHeight;
            inline_content.setWidth(w);
            inline_content.setHeight(h);
        }, this);

        /* ********************** INFERENCE PAGE *****************************/

        /* ********************** CREATE ELEMENTS ****************************/

        // main container (contains outer splitpane and control panel)
        var ctr_inference = new qx.ui.container
                                     .Composite(new qx.ui.layout.VBox());

        // outer splitpane (contains inference settings and graph
        // visualization container)
        var splitpane = new qx.ui.splitpane.Pane("horizontal");
        this._splitpane = splitpane;

        // container for inference settings form
        var ctr_infsettings = new qx.ui.container
                                            .Composite(new qx.ui.layout
                                                                .VBox()).set({
            minWidth: .2*document.getElementById("page", true, true).offsetWidth
        });
        this._ctr_infsettings = ctr_infsettings;

        // form showing the settings and mln/db contents used in inference
        var ctr_formgrp = this.buildForm();
        var ctr_placeholder = new qx.ui.container.Composite();
        ctr_placeholder.setHeight(100);

        // container for visualization elements
        var ctr_graph_visualization = new qx.ui.container
                                         .Composite(new qx.ui.layout
                                                             .Canvas()).set({
            width: .79*document.getElementById("page", true, true).offsetWidth
        });
        this._ctr_graph_visualization = ctr_graph_visualization;

        // conditional probability img
        // (source png will be set in each inf step)
        var img_condprob = new qx.ui.basic.Image();
        img_condprob.setAllowGrowX(true);
        img_condprob.setAllowShrinkX(true);
        img_condprob.setAllowGrowY(true);
        img_condprob.setAllowShrinkY(true);
        img_condprob.setScale(true);
        this._img_condprob = img_condprob;

        // window showing conditional probability of current inference step
        var img_condprobWin = new qx.ui.window.Window("Conditional Probability");
        img_condprobWin.setWidth(.2*document.getElementById("page",
                                                        true,
                                                        true).offsetWidth);
        img_condprobWin.setHeight(.1*document.getElementById("page",
                                                         true,
                                                         true).offsetWidth);
        img_condprobWin.setShowMinimize(false);
        img_condprobWin.setLayout(new qx.ui.layout.Canvas());
        img_condprobWin.setContentPadding(4);
        img_condprobWin.add(img_condprob);
        img_condprobWin.open();
        img_condprobWin.hide();
        this._img_condprobWin = img_condprobWin;

        // pop up message flashing to notify user
        var html_flashmsg = new qx.ui.embed.Html();
        html_flashmsg.setWidth(500);
        html_flashmsg.setHeight(300);
        html_flashmsg.setMarginLeft(-250);
        html_flashmsg.setMarginTop(-150);
        html_flashmsg.setOpacity(0);
        html_flashmsg.hide();
        this._html_flashmsg = html_flashmsg;

        // window showing console log msgs from xhr requests
        var win_logging = new qx.ui.window.Window("Log");
        win_logging.setWidth(500);
        win_logging.setHeight(300);
        win_logging.setShowMinimize(false);
        win_logging.setLayout(new qx.ui.layout.Grow());
        this._txtarea_log = new qx.ui.form.TextArea("").set({
            font: qx.bom.Font.fromString("14px monospace")
        });
        win_logging.add(this._txtarea_log);
        this.getRoot().add(win_logging, {left:50, top:50});
        this._win_logging = win_logging;

        // overlay image indicating work in progress
        var img_wait_inf = new qx.ui.basic.Image();
        img_wait_inf.setSource('/prac/static/images/wait.gif');
        img_wait_inf.getContentElement().setAttribute('id', 'waitImgInf');
        img_wait_inf.setWidth(300);
        img_wait_inf.setHeight(225);
        img_wait_inf.setMarginLeft(-150);
        img_wait_inf.setMarginTop(-125);
        img_wait_inf.setScale(1);
        img_wait_inf.hide();
        this._img_wait_inf = img_wait_inf;

        var img_inferencelogo = new qx.ui.basic.Image();
        img_inferencelogo.setSource('/prac/static/images/prac-darkonbright-transp.png');
        img_inferencelogo.getContentElement().setAttribute('id', 'img_inferencelogo');
        img_inferencelogo.setWidth(220);
        img_inferencelogo.setHeight(95);
        img_inferencelogo.setScale(1);
        this._img_inferencelogo = img_inferencelogo;

        var ctr_visualization = new qx.ui.container.Composite()
        ctr_visualization.setLayout(new qx.ui.layout.Grow());
        ctr_visualization.getContentElement().setAttribute('id', 'viz');

        var ctr_flowchart = new qx.ui.container
                                          .Composite(new qx.ui.layout.Grow())
                                          .set({ minWidth: 270,
                                                 minHeight: 310
                                          });
        ctr_flowchart.getContentElement().setAttribute('id', 'flowchart');
        this._ctr_flowchart = ctr_flowchart;

        // control pane (containing text field, buttons and checkboxes used
        // for inference)
        var grp_pracinfcontrols = this.buildControlPane();
        this._grp_pracinfcontrols = grp_pracinfcontrols;

        var win_cramplans = new qx.ui.window.Window('Cram Plans');
        win_cramplans.setWidth(900);
        win_cramplans.setHeight(300);
        win_cramplans.setShowMinimize(false);
        win_cramplans.setLayout(new qx.ui.layout.Grow());
        //this._txtarea_cramplans = new qx.ui.form.TextArea("").set({
        //    font: qx.bom.Font.fromString("14px monospace")
        //});
        this._txtarea_cramplans = new qx.ui.embed.Html("");
        //this._txtarea_cramplans.setCssClass("formatcp");
        // begin PRAC2CRAM extension
        var button_cram = new qx.ui.form.Button("Execute");

        button_cram.addListener("execute", function (e) {

            cramPlan = this._txtarea_cramplans.getValue();
            cramPlan = cramPlan.trim();
            var updatedText = cramPlan + "\nSending CRAM plan to execute...";

            this._txtarea_cramplans.setValue(updatedText);

            var req = new qx.io.request.Xhr('/prac/_execute_plan', 'POST');
            req.setRequestHeader("Content-Type", "application/json");
            // we on't need the fake plan string at the moment, but it could come handy later
            // so it's included in the data but not send to the service
            req.setRequestData({ 'plan': cramPlan});
            var that = this;
            // when the ROS service call returns success
            req.addListener("success", function(e) {

               var tar = e.getTarget();
               var response = tar.getResponse();
               that._notify(response.message, 500);
               if (response.status == 0) {

               // switch automatically to gazebo tab
               that.__tabView.setSelection([this.__page_gzweb]); //  must be given as a list with one element
               // make the CRAM plan window smaller
               that.__win_cramplans.setWidth(400);
               var h = document.getElementById("page", true, true).offsetHeight;
               that.__win_cramplans.moveTo(50, (h-500));
               }
            }, this);
            req.send();
        }, this);

        this.__button_cram = button_cram;

        win_cramplans.add(this._txtarea_cramplans,  {left:"0%", top:"0%", right:"0%", bottom:" 15%"});
        win_cramplans.add(button_cram,  {right:"0%", bottom:"1%", width:"20%"});

        this.getRoot().add(win_cramplans, {left:10, top:10});
        this.__win_cramplans = win_cramplans;

        // end PRAC2CRAM extension


        /* ************************ LISTENERS ********************************/

        win_logging.addListener("close", function() {
            this._chkbx_showlog.setValue(false);
        }, this);

        img_condprobWin.addListener("close", function() {
            this._chkbx_show_condprob.setValue(false);
        }, this);

        // resize image to fit in window
        img_condprobWin.addListener("resize", function(e) {
            var ratio =  typeof this._imgRatio != 'undefined' ?
                                                  this._imgRatio :
                                                  1;
            var newWidth = e.getData().width - 10;
            var newHeight = e.getData().height - 30;

            if (newWidth / ratio <= newHeight) {
                newHeight = newWidth / ratio;
            } else {
                newWidth = newHeight * ratio;
            }
            img_condprob.setWidth(parseInt(newWidth, 10));
            img_condprob.setHeight(parseInt(newHeight, 10));
        }, this);

        // resize image to fit in window
        img_condprob.addListener("changeSource", function(e) {
            var ratio =  typeof this._imgRatio != 'undefined'? this._imgRatio : 1;
            var newWidth = img_condprobWin.getInnerSize().width - 10;
            var newHeight = img_condprobWin.getInnerSize().height - 30;

            if (newWidth / ratio <= newHeight) {
                newHeight = newWidth / ratio;
            } else {
                newWidth = newHeight * ratio;
            }
            img_condprob.setWidth(parseInt(newWidth, 10));
            img_condprob.setHeight(parseInt(newHeight, 10));
        }, this);

        // reposition graph when inference settings are shown/hidden
        ctr_graph_visualization.addListener('resize', function(e) {
            if (typeof this._graph != 'undefined') {
                var vizSize = ctr_graph_visualization.getInnerSize();
                var bounds = ctr_graph_visualization.getBounds();
                this._graph.w = vizSize.width;
                this._graph.h = vizSize.height;
                this._graph.update();
            }
        }, this);

        /* ********************** SET UP LAYOUT ******************************/

        ctr_infsettings.add(ctr_placeholder);
        ctr_infsettings.add(ctr_formgrp);
        ctr_graph_visualization.add(ctr_flowchart, { right: 0,
                                                    top: 0,
                                                    width: "20%",
                                                    height:"auto"});
        ctr_graph_visualization.add(ctr_visualization, { left: 0,
                                              top: 0,
                                              width: "100%",
                                              height:"100%"});
        ctr_graph_visualization.add(img_wait_inf, { left: "50%", top: "50%"});
        ctr_graph_visualization.add(img_inferencelogo, { left: 5, top: 5});
        splitpane.add(ctr_infsettings, {width: "20%"});
        splitpane.add(ctr_graph_visualization);
        ctr_inference.add(splitpane, {flex: 1, width: "100%"});
        ctr_inference.add(grp_pracinfcontrols, {width: "100%"});

        /* ********************** BROWSER PAGE *******************************/

        /* ********************** CREATE ELEMENTS ****************************/

        var img_browserlogo = new qx.ui.basic.Image();
        img_browserlogo.setSource('/prac/static/images/prac-darkonbright-transp.png');
        img_browserlogo.getContentElement().setAttribute('id', 'img_inferencelogo');
        img_browserlogo.setWidth(220);
        img_browserlogo.setHeight(95);
        img_browserlogo.setScale(1);
        this._img_browserlogo = img_browserlogo;

        // overlay image indicating work in progress
        var img_wait_br = new qx.ui.basic.Image();
        img_wait_br.setSource('/prac/static/images/wait.gif');
        img_wait_br.getContentElement().setAttribute('id', 'waitImgInf');
        img_wait_br.setWidth(300);
        img_wait_br.setHeight(225);
        img_wait_br.setMarginLeft(-150);
        img_wait_br.setMarginTop(-125);
        img_wait_br.setScale(1);
        img_wait_br.hide();
        this._img_wait_br = img_wait_br;

        // main container (contains outer menu widget and graph widget)
        var ctr_browser = new qx.ui.container
                                        .Composite(new qx.ui.layout.HBox());

        var ctr_browsermenu = new qx.ui.container
                                  .Composite(new qx.ui.layout.VBox());
        ctr_browsermenu.setAlignX('right');
        ctr_browsermenu.setMinWidth(400);

        // action core selectbox
        var layout_ctr_ac = new qx.ui.layout.Grid();
        layout_ctr_ac.setColumnWidth(0, 100);
        layout_ctr_ac.setColumnAlign(0, 'right', 'center');
        layout_ctr_ac.setColumnWidth(1, 300);
        layout_ctr_ac.setColumnAlign(1, 'left', 'center');
        var ctr_ac = new qx.ui.container.Composite(layout_ctr_ac);
        ctr_ac.setMarginTop(50);

        var sel_ac = new qx.ui.form.SelectBox();
        this._sel_ac = sel_ac;
        ctr_ac.add(new qx.ui.basic
                                  .Label('<b>Action Core: </b>')
                                  .set({rich: true}), {row: 0, column: 0});
        ctr_ac.add(sel_ac, {row: 0, column: 1});

        // container for role selectboxes to choose wordnet concepts from
        var layout_ctr_rolesconcepts = new qx.ui.layout.Grid();
        layout_ctr_rolesconcepts.setColumnWidth(0, 100);
        layout_ctr_rolesconcepts.setColumnAlign(0, 'right', 'center');
        layout_ctr_rolesconcepts.setColumnWidth(1, 300);
        layout_ctr_rolesconcepts.setColumnAlign(1, 'left', 'center');
        var ctr_rolesconcepts = new qx.ui.container.Composite(layout_ctr_rolesconcepts);
        this._ctr_rolesconcepts = ctr_rolesconcepts;

        // button to trigger getting role distributions for selection
        var btn_sel_roledistr = new qx.ui.form.Button("Compute");
        btn_sel_roledistr.setMaxWidth(300);
        btn_sel_roledistr.setAlignX('right');
        this._btn_sel_roledistr = btn_sel_roledistr;

        // role selectbox to update distribution svg
        var layout_ctr_roles = new qx.ui.layout.Grid();
        layout_ctr_roles.setColumnWidth(0, 100);
        layout_ctr_roles.setColumnAlign(0, 'right', 'center');
        layout_ctr_roles.setColumnWidth(1, 300);
        layout_ctr_roles.setColumnAlign(1, 'left', 'center');
        var ctr_roles = new qx.ui.container.Composite(layout_ctr_roles);
        ctr_roles.setMarginTop(20);

        var sel_role = new qx.ui.form.SelectBox();
        this._sel_role = sel_role;

        ctr_roles.add(btn_sel_roledistr, {row: 0, column: 1})
        ctr_roles.add(new qx.ui.basic.Label(''), {row: 1, column: 0});
        ctr_roles.add(new qx.ui.basic
                                  .Label('<b>Role: </b>')
                                  .set({rich: true}), {row: 2, column: 0});
        ctr_roles.add(sel_role, {row: 2, column: 1});


        // gets a value in init. contains the 17.*** wordnet concepts the
        // role selectboxes are being filled with
        this.__wordnetconcepts = [];

        // canvas for drawing distribution svg
        var layout_ctr_distrsvg = new qx.ui.layout.VBox()
        var ctr_distrsvg = new qx.ui.container.Composite();
        this._ctr_distrsvg = ctr_distrsvg;
        ctr_distrsvg.setLayout(layout_ctr_distrsvg);
        ctr_distrsvg.set({
            width: document.getElementById("page", true, true).offsetWidth - 450
        });

        this.__zoom = 3;
        var slider_distr = new qx.ui.form.Slider('horizontal').set({
            minimum: 100,
            maximum: 100 * this.__zoom,
            value: 0
        })
        slider_distr.setMaxHeight(100);
        this._slider_distr = slider_distr;

        var grp_slider =
        {
            slider: slider_distr,
            minimum: new qx.ui.basic.Label("Min: " + slider_distr.getMinimum().toString() + '%'),
            maximum: new qx.ui.basic.Label("Max: " + slider_distr.getMaximum().toString() + '%'),
            value: new qx.ui.basic.Label('Zoom')
        };
        grp_slider.value.setTextAlign("center");
        this._grp_slider = grp_slider;

        var layout_ctr_slider = new qx.ui.layout.Grid();
        var ctr_slider = new qx.ui.container.Composite(layout_ctr_slider);

        ctr_slider.setPadding(20);
        ctr_slider.setWidth(400);

        layout_ctr_slider.setSpacing(5);
        layout_ctr_slider.setColumnFlex(0, 1);
        layout_ctr_slider.setColumnFlex(1, 1);
        layout_ctr_slider.setColumnFlex(2, 1);

        layout_ctr_slider.setColumnAlign(0, "left", "bottom");
        layout_ctr_slider.setColumnAlign(1, "center", "bottom");
        layout_ctr_slider.setColumnAlign(2, "right", "bottom");

        ctr_slider.add(grp_slider.minimum, {row: 0, column: 0});
        ctr_slider.add(grp_slider.value, {row: 0, column: 1});
        ctr_slider.add(grp_slider.maximum, {row: 0, column: 2});

        ctr_slider.add(grp_slider.slider, {row: 1, column: 0, colSpan: 3, rowSpan: 1});

        var ctr_canvas_distribution = new qx.ui.container
                                               .Composite(new qx.ui.layout.Canvas);

        var scrollctr_html_distr = new qx.ui.container.Scroll().set({
//            backgroundColor: "yellow",
            width: document.getElementById("page", true, true).offsetWidth - 450,
            height: document.getElementById("page", true, true).offsetHeight - 120
        });
        scrollctr_html_distr.getContentElement().setAttribute("id","scrollctr_html_distr");
        this._scrollctr_html_distr = scrollctr_html_distr;

        var html_distr = new qx.ui.embed.Html();
        html_distr.getContentElement().setAttribute("id","html_distr");
        this.__html_distr = html_distr;


        /* ********************** LISTENERS **********************************/

        btn_sel_roledistr.addListener("execute", this.get_dists, this);
        slider_distr.addListener("changeValue", this.dist_zoom, this);
        sel_ac.addListener("changeSelection", this.change_ac ,this);
        sel_role.addListener("changeSelection", this.change_distr ,this);

        /* ********************** SET UP LAYOUT ******************************/

        // left column (selections)
        ctr_browsermenu.add(img_browserlogo);
        ctr_browsermenu.add(ctr_ac);
        ctr_browsermenu.add(ctr_rolesconcepts);
        ctr_browsermenu.add(ctr_roles);

        scrollctr_html_distr.add(html_distr);

        ctr_canvas_distribution.add(scrollctr_html_distr);
        ctr_canvas_distribution.add(img_wait_br, { left: "50%", top: "50%"});

        ctr_distrsvg.add(ctr_slider);
        ctr_distrsvg.add(ctr_canvas_distribution);

        ctr_browser.add(ctr_browsermenu, {height: "100%"});
        ctr_browser.add(ctr_distrsvg, {height: "100%"});

        /* ********************** SET UP MAIN LAYOUT *************************/

        var tabview = new qx.ui.tabview.TabView('bottom');
        tabview.setContentPadding(2,2,2,2);
        this.__tabview = tabview;

        ////////////////// INFERENCE PAGE ////////////////////
        var page_inference = new qx.ui.tabview.Page("Inference");
        this.__page_inference = page_inference;
        page_inference.setLayout(new qx.ui.layout.Grow());
        page_inference.add(ctr_inference, {width: "100%", height: "100%"});
        tabview.add(page_inference, {width: "100%", height: "100%"});

        // begin PRAC2CRAM extension
        ////////////////// GAZEBO (GZWEB) PAGE ////////////////////
        var iframe_gzweb = new qx.ui.embed.Iframe("/gzweb");
        var ctr_gzweb = new qx.ui.container.Composite(new qx.ui.layout.Grow());
        var page_gzweb = new qx.ui.tabview.Page("Gazebo Simulation");
        this.__page_gzweb = page_gzweb;
        page_gzweb.setLayout(new qx.ui.layout.Grow()); // Grow layout because there'll be only one child
        ctr_gzweb.add(iframe_gzweb);
        page_gzweb.add(ctr_gzweb, {width: "100%", height: "100%"});
        tabview.add(page_gzweb, {width: "100%", height: "100%"});
        // end PRAC2CRAM extension

        ////////////////// BROWSER PAGE ////////////////////
        var page_browser = new qx.ui.tabview.Page("Browser");
        this.__page_browser = page_browser;
        page_browser.setLayout(new qx.ui.layout.Grow());
        page_browser.add(ctr_browser, {width: "100%", height: "100%"});
        tabview.add(page_browser, {width: "100%", height: "100%"});

        ctr_main.add(tabview, {width: "100%", height: "100%"});
        ctr_main.add(html_flashmsg, { left: "50%", top: "50%"});

        ctr_mainscroll.add(ctr_main, {width: "100%",
                                                      height: "100%"});
        inline_content.add(ctr_mainscroll, {width: "100%", height: "100%"});

        /* *************************** INIT **********************************/

        this.init();
        this.load_flow_chart();
        this.getRoot().add(img_condprobWin, {left:20, top:20});
        this.__var_show_ctr_infsettings = false;
        this.__var_infer_stepwise = false;
        this.__var_use_chkbx_acatontology = false;
        this.change_visibility();
        this.send_user_stats();
    },


    /**
     * load initial graph
     */
    loadGraph : function() {
        if (typeof this._graph === 'undefined') {
            this._graph = new pracweb.Graph();
            var vizSize = this._ctr_graph_visualization.getInnerSize();
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
    buildControlPane : function() {

        /* ********************** CREATE ELEMENTS ****************************/

        var grp_main = new qx.ui.groupbox.GroupBox("PRAC Inference");
        var layout_grp_main = new qx.ui.layout.HBox(20);
        grp_main.setLayout(layout_grp_main);

        // combobox containing nl combo_descrs
        var combo_descr = new qx.ui.form.ComboBox();
        combo_descr.setWidth(500);
        combo_descr.add(new qx.ui.form.ListItem("start the centrifuge."));
        combo_descr.add(new qx.ui.form.ListItem("Add some water to the purine."));
        combo_descr.add(new qx.ui.form.ListItem("Add some arsenic_acid to the imidazole."));
        combo_descr.add(new qx.ui.form.ListItem("Add 5 drops of the lysergic_acid to the pyrimidine."));
        combo_descr.add(new qx.ui.form.ListItem("Add 1 liter of water to the chlorous_acid."));
        combo_descr.add(new qx.ui.form.ListItem("Neutralize the methacrylic_acid with 100 milliliters of cyanuramide."));
        combo_descr.add(new qx.ui.form.ListItem("start with neutralizing the pyridine with 4 drops of hydrofluoric_acid."));
        combo_descr.add(new qx.ui.form.ListItem("Open the drawer."));
        combo_descr.add(new qx.ui.form.ListItem("Open the fridge."));
        combo_descr.add(new qx.ui.form.ListItem("Open the test_tube."));
        combo_descr.add(new qx.ui.form.ListItem("Open the cupboard."));
        combo_descr.add(new qx.ui.form.ListItem("Flavour the tomato_sauce with the oregano."));
        combo_descr.add(new qx.ui.form.ListItem("Fill a cup with water."));
        combo_descr.add(new qx.ui.form.ListItem("Preheat the oven to 100 degree_fahrenheit."));
        combo_descr.add(new qx.ui.form.ListItem("Fill a blender with apples."));
        combo_descr.add(new qx.ui.form.ListItem("Fill a mixer with pineapples."));
        combo_descr.add(new qx.ui.form.ListItem("Fill a glass with wine."));
        combo_descr.add(new qx.ui.form.ListItem("Add some cheese to the pizza."));
        combo_descr.setValue(combo_descr.getChildrenContainer().getSelectables()[0].getLabel());

        var chkbx_show_infsettings = new qx.ui.form.CheckBox("Show Inference settings");
        var chkbx_infer_stepwise = new qx.ui.form.CheckBox("Step-by-step inference");
        var btn_run_inference = new qx.ui.form.Button("Run Inference",
                                     "/prac/static/images/resultset_next.png");
        this._btn_run_inference = btn_run_inference;
        var btn_next_infstep = new qx.ui.form.Button("Next Step",
                                     "/prac/static/images/resultset_last.png");
        this._btn_next_infstep = btn_next_infstep;
        btn_next_infstep.setEnabled(false);

        // taxonomy visualization
        var win_taxonomy = new qx.ui.window.Window("Wordnet Taxonomy");
        win_taxonomy.setWidth(700);
        win_taxonomy.setHeight(500);
        win_taxonomy.setShowMinimize(false);
        win_taxonomy.setLayout(new qx.ui.layout.Grow());
        var taxCanvas = new qx.ui.embed.Html();
        win_taxonomy.add(taxCanvas);
        this.getRoot().add(win_taxonomy, {left:20, top:20});

        var btn_show_taxonomy = new qx.ui.form.Button("Show Taxonomy");

        var btn_get_roledists = new qx.ui.form.Button("Get Role Distributions");
        btn_get_roledists.setEnabled(false);
        this._btn_get_roledists = btn_get_roledists;

        var chkbx_show_flowchart = new qx.ui.form.CheckBox("Show/hide Flowchart");
        chkbx_show_flowchart.setValue(true);

        var chkbx_show_condprob = new qx.ui.form.CheckBox("Show/hide Cond. Probability");
        this._chkbx_show_condprob = chkbx_show_condprob;

        var chkbx_showlog = new qx.ui.form.CheckBox("Show/hide Log");
        this._chkbx_showlog = chkbx_showlog;

        var chkbx_acatontology = new qx.ui.form.CheckBox("Use ACAT ontology");

        /* ********************** LISTENERS **********************************/

        combo_descr.addListener("keydown", function(e) {
            this._btn_run_inference.setEnabled(true);
            this.clear_flow_chart();
            document.getElementById('init').nextElementSibling.style.fill = "#bee280";
        }, this);

        combo_descr.addListener("changeValue", function(e) {
            this._btn_run_inference.setEnabled(true);
            this.clear_flow_chart();
            document.getElementById('init').nextElementSibling.style.fill = "#bee280";
        }, this);

        chkbx_show_infsettings.addListener("changeValue", function(e) {
            this.__var_show_ctr_infsettings = e.getData();
            this.change_visibility();
        }, this);

        // toggling Step-by-step check box will enable/disable 'Next'-button
        // and set the inference type
        chkbx_infer_stepwise.addListener("changeValue", function(e) {
            var that = this;
            that.__var_infer_stepwise = e.getData();
        }, this);

        // trigger the PRAC inference
        btn_run_inference.addListener('execute', function() {
            if (this.__var_infer_stepwise) {
                this._btn_run_inference.setEnabled(false);
            }
            btn_get_roledists.setEnabled(false);
            this.loadGraph();
            this.clear_flow_chart();
            this._next_module = 'nl_parsing';
            document.getElementById('init').nextElementSibling.style.fill = "#bee280";
            this._oldRes = {};
            this._oldEvidence = combo_descr.getValue();
            var req = this.start_inference("POST");
            req.setRequestHeader("Content-Type", "application/json");
            req.setRequestData({ 'sentence': combo_descr.getValue(),
                                 'acatontology': this.__var_use_chkbx_acatontology });
            req.send();
        }, this);

        // 'Next'-button will trigger new inference step
        btn_next_infstep.addListener('execute', function(e) {
            var that = this;
            var req = that.start_inference("GET");
                req.send();
        }, this);

        btn_show_taxonomy.addListener("execute", function() {
            var req = new qx.io.request.Xhr();
            req.setUrl("/prac/_get_wordnet_taxonomy");
            req.setMethod("GET");

            req.addListener("success", function(e) {
                var tar = e.getTarget();
                var response = tar.getResponse();
                taxCanvas.setHtml(response);
                win_taxonomy.open();
            });
            req.send();
        }, this);

        btn_get_roledists.addListener('execute', this.get_role_distributions, this);

        chkbx_show_flowchart.addListener('changeValue', function(e) {
            var el = this._ctr_flowchart.getContentElement().getDomElement();
            if (e.getData())
                this._ctr_flowchart.show();
            else
                this._ctr_flowchart.hide();
        }, this);

        chkbx_show_condprob.addListener('changeValue', function(e) {
          if (e.getData())
            this._img_condprobWin.show();
          else
            this._img_condprobWin.hide();
        }, this);

        chkbx_showlog.addListener('changeValue', function(e) {
          if (e.getData())
            this._win_logging.show();
          else
            this._win_logging.hide();
        }, this);

        chkbx_acatontology.addListener("changeValue", function(e) {
            var that = this;
            that.__var_use_chkbx_acatontology = e.getData();
        }, this);

        /* ********************** SET UP LAYOUT ******************************/

        var ctr_instruction = new qx.ui.container
                                       .Composite(new qx.ui.layout.VBox());
        ctr_instruction.add(new qx.ui.basic
                                     .Label("Instruction:"));
        ctr_instruction.add(combo_descr);

        var layout_ctr_infbuttons = new qx.ui.layout.Grid();
        layout_ctr_infbuttons.setColumnWidth(0, 130);
        layout_ctr_infbuttons.setColumnWidth(1, 130);
        var ctr_infbuttons = new qx.ui.container.Composite(layout_ctr_infbuttons);

        ctr_infbuttons.add(btn_run_inference, {row: 0, column: 0});
        ctr_infbuttons.add(btn_show_taxonomy, {row: 0, column: 1});
        ctr_infbuttons.add(btn_next_infstep, {row: 1, column: 0});
        ctr_infbuttons.add(btn_get_roledists, {row: 1, column: 1});

        var slide_options = new qx.ui.container.SlideBar();
        slide_options.setLayout(new qx.ui.layout.HBox(5));
        slide_options.add(chkbx_show_infsettings);
        slide_options.add(chkbx_infer_stepwise);
        slide_options.add(chkbx_show_flowchart);
        slide_options.add(chkbx_show_condprob);
        slide_options.add(chkbx_acatontology);
        slide_options.add(chkbx_showlog);

        grp_main.add(ctr_instruction, {flex: 2});
        grp_main.add(ctr_infbuttons);
        grp_main.add(slide_options, {flex: 2});
        return grp_main;
    },


    /**
    * Start the inference process
    */
    start_inference : function(e) {
       this.show_wait_animation('inf', true);

       var req = new qx.io.request.Xhr("/prac/_start_inference", e);
       req.setRequestHeader("Content-Type", "application/json");
       req.addListener("success", function(e) {
               var that = this;
               var tar = e.getTarget();
               var response = tar.getResponse();
               this.notify(response.message, 100);
               this.get_inference_status();
       }, this);
       return req;
    },


    /**
    * Request inference status
    */
    get_inference_status : function() {

        this.show_wait_animation('inf', true);
        this._btn_next_infstep.setEnabled(false);
        this.__win_cramplans.close();

        // update flowchart
        if (this._next_module === 'achieved_by' ||
           (this._next_module === 'plan_generation') &&
           (this._last_module != 'plan_generation')) {
            var that = this;
            // because following timeout will cause _next_module to be
            // overwritten too quickly:
            var tmpNM = that._next_module;
            this.clear_flow_chart();
            document.getElementById('executable').nextElementSibling
                                                 .style.fill = "#bee280";
            setTimeout( function() {
                that.clear_flow_chart();
                document.getElementById(tmpNM).nextElementSibling
                                              .style.fill = "#bee280";
            }, 1000);
        } else {
            this.clear_flow_chart();
            document.getElementById(this._next_module).nextElementSibling
                                                      .style.fill = "#bee280";
        }

        var req = new qx.io.request.Xhr("/prac/_get_status", "POST");
        req.setRequestHeader("Content-Type", "application/json");
        req.addListener("success", function(e) {
            var that = this;
            var tar = e.getTarget();
            var response = tar.getResponse();

            var message = response.message;

            if (response.status == true) {
                this._txtarea_log.setValue(response.log);
                this._txtarea_log.getContentElement().scrollToY(100000);

                var responseResult = response.result;
                var responseSettings = response.settings == null ?
                                                            {} :
                                                            response.settings;
                that.set_exp_settings(responseSettings);


                // determine links to be removed/added
                var updateLinks = that.calculateRedrawing(that._oldRes,
                                                           responseResult);
                var idle_time = 1000 +
                                (updateLinks[0].length +
                                updateLinks[1].length) *
                                that._graph.WAITMSEC;
                that._oldRes = responseResult;
                that._oldEvidence = responseSettings == null ?
                                                        '' :
                                                        responseSettings['evidence'];

                that.show_wait_animation('inf', false);

                if (response.finish) {
                    console.log(" I am DONE! ");
                    that.show_wait_animation('inf', true);
                    that.updateGraph(updateLinks[0], updateLinks[1]);

                    // wait 3 seconds, then clear flowchart
                    setTimeout( function() {
                        that.get_cram_plan();
                    }, idle_time);

                    that._btn_next_infstep.setEnabled(false);
                    that._btn_run_inference.setEnabled(true);
                    that._last_module = '';
                    that.show_wait_animation('inf', false);
                } else if (that._next_module === 'plan_generation') {
                    // do not redraw graph because plan_generation does
                    // not update output_dbs
                    that._last_module = 'plan_generation';
                    var req = that.start_inference("GET");
                    req.send();
                } else {
                    that._last_module = that._next_module;
                    that._next_module = that.get_next_module();
                    that.get_cond_prob();
                    that.updateGraph(updateLinks[0], updateLinks[1]);
                    setTimeout( function() {
                        that._btn_next_infstep.setEnabled(true);
                        // set enabled when senses_and_roles has finished
                        if (that._last_module == 'senses_and_roles') {
                          that._btn_get_roledists.setEnabled(true);
                        }
                    }, idle_time);

                    if (!that.__var_infer_stepwise) {
                        console.log('bumming around for',
                                    idle_time,
                                    ' mseconds before sending new request...');
                        setTimeout( function() {
                            var req = that.start_inference("GET");
                            req.send();
                        }, idle_time); // wait for graph to be updated
                    }
                }
            } else {
                this.get_inference_status();
            }
            this.notify(message, 100);
        }, this);
        req.send();
    },


    /**
     * get name of the next module to be executed
     */
    get_next_module : function() {
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
    get_modules : function() {
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
              this.moduleSelect.add(new qx.ui.form
                                             .ListItem(response.modules[i]));
            }

            for (var i = 0; i < response.methods.length; i++) {
              this.methodSelect.add(new qx.ui.form
                                             .ListItem(response.methods[i][1],
                                                       null,
                                                       response.methods[i][0]));
            }
        }, this);
        moduleReq.send();
    },


    /**
     * request conditional probability png for current inference step
     * and display it under the flowchart
     */
    get_cond_prob : function() {
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
            this._img_condprob.resetSource();

            if (response.img !== '') {
                this._img_condprob.setSource('data:image/png;base64,' +
                                         response.img);
            }
            return;
        }, this);
        moduleReq.send();
    },

    /**
     * calculate the links to be added and the links
     * to be removed from the visualization graph
     */
    calculateRedrawing : function(oldRes, newRes) {
        var toBeRemoved = [];
        var toBeAdded = [];
        var remove;
        var add;

        // old links to be removed
        for (var i = 0; i < oldRes.length; i++) {
            remove = true;

            for (var j = 0; j < newRes.length; j++) {
                // if there is already a link between the nodes,
                // do not remove it
                if (oldRes[i].source.name === newRes[j].source.name &&
                    oldRes[i].target.name === newRes[j].target.name &&
                    oldRes[i].value === newRes[j].value) {
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
                if (newRes[i].target.name === oldRes[j].target.name &&
                    newRes[i].source.name === oldRes[j].source.name &&
                    newRes[i].value === oldRes[j].value) {
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
                if (oldRes[i].target.name === newRes[j].target.name &&
                    oldRes[i].source.name === newRes[j].source.name &&
                    oldRes[i].value === newRes[j].value) {
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
                if (newRes[i].target.name === oldRes[j].target.name &&
                    newRes[i].source.name === oldRes[j].source.name &&
                    newRes[i].value === oldRes[j].value) {
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
    buildForm : function() {
        /* ********************** CREATE ELEMENTS ****************************/

        var grouplayout = new qx.ui.layout.VBox();
        var group = new qx.ui.container.Composite(grouplayout);

        var formLayout = new qx.ui.layout.Grid();
        formLayout.setColumnWidth(0, 130); // col 0 has width 130
        formLayout.setColumnWidth(2, 270); // col 2 has width 270
        formLayout.setColumnAlign(0, "left", "middle");
        formLayout.setColumnAlign(1, "left", "middle");
        var formgroup = new qx.ui.container.Composite(formLayout);

        //  form widgets
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
        var mlnAreaContainer = new qx.ui.container
                                        .Composite(mlnAreaContainerLayout);

        var textAreaMLN = new qx.ui.form.TextArea('');
        this.__textAreaMLN = textAreaMLN;
        textAreaMLN.setMinWidth(300);
        textAreaMLN.getContentElement().setAttribute('id', 'mlnArea');

        var evidenceLabel = new qx.ui.basic.Label().set({
            value: this._template('Evidence:', 'label'),
            rich: true
        });
        var evidenceContainerLayout = new qx.ui.layout.Grow();
        var evidenceContainer = new qx.ui.container
                                         .Composite(evidenceContainerLayout);
        var textAreaEvidence = new qx.ui.form.TextArea('');
        this.__textAreaEvidence = textAreaEvidence;
        textAreaEvidence.setMinWidth(300);
        textAreaEvidence.getContentElement().setAttribute('id',
                                                          'evidenceArea');

        /* ********************** LISTENERS **********************************/

        textAreaMLN.addListener('appear', function() {
            this.highlight(textAreaMLN.getContentElement()
                                       .getAttribute('id'));
        }, this);

        textAreaEvidence.addListener("appear", function() {
            this.highlight(textAreaEvidence.getContentElement()
                                            .getAttribute('id'));
        }, this);

        /* ********************** SET UP LAYOUT ******************************/

        mlnAreaContainer.add(this.__textAreaMLN);
        evidenceContainer.add(textAreaEvidence);

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
    get_role_distributions : function(e) {
        console.log('getting role distributions...');
        this.show_wait_animation('inf', true);
        var req = new qx.io.request.Xhr();
        req.setUrl("/prac/_get_role_distributions");
        req.setMethod('GET');
        req.setRequestHeader("Cache-Control", "no-cache");
        req.setRequestHeader("Content-Type", "application/json");
        var that = this;
        req.addListener("success", function(e) {
            this.show_wait_animation('inf', false);
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

        req.addListener("fail", function(e) {
            this.show_wait_animation('inf', false);
            this.notify("Error! Could not generate Role Distributions.", 100);
        }, that);
        req.send();
    },


    /**
     * request cram plan for instruction and open it in new window
     */
    get_cram_plan : function(e) {
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
                for (var i = 0; i < response.plans.length; i++) {
                    response.plans[i] = formatCP(response.plans[i]);
                }
                this._txtarea_cramplans.setHtml(response.plans.join(''));
                this.__win_cramplans.open();
                return;
            }
        }, that);
        req.send();
    },


    /**
     * load flowchart svg into embedding
     */
    load_flow_chart : function(e) {
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
            this._ctr_flowchart.getContentElement().getDomElement().innerHTML = response;
            return;
        }, that);
        req.send();
    },


    /**
     * clear flow chart
     */
    clear_flow_chart : function(e) {
        var nodes = ['init','nl_parsing','ac_recognition','senses_and_roles','executable','plan_generation','achieved_by','roles_transformation'];

        for (var x = 0; x < nodes.length; x++) {
            document.getElementById(nodes[x]).nextElementSibling.style.fill = "white";
        }
    },


    /**
     * update values for inference settings of current step
     */
    set_exp_settings : function(settings) {
        typeof settings['module'] != 'undefined' ? this.moduleSelect.setValue(this._template(settings['module'])) : this.moduleSelect.resetValue();
        typeof settings['logic'] != 'undefined' ? this.logicSelect.setValue(this._template(settings['logic'])) : this.logicSelect.resetValue();
        typeof settings['method'] != 'undefined' ? this.methodSelect.setValue(this._template(settings['method'])) : this.methodSelect.resetValue();
        typeof settings['queries'] != 'undefined' ? this.queriesField.setValue(this._template(settings['queries'])) : this.queriesField.resetValue();
        typeof settings['cwPreds'] != 'undefined' ? this.cwPredsField.setValue(this._template(settings['cwPreds'])) : this.cwPredsField.resetValue();
        typeof settings['closedWorld'] != 'undefined' ? this.closedWorld.setValue(this._template(settings['closedWorld'] ? 'true' : 'false')) : this.closedWorld.resetValue();

        var mlnContent = typeof settings['mln'] === 'undefined' ? '' : settings['mln'];
        this.__textAreaMLN.setValue(mlnContent);
        this.highlight(this.__textAreaMLN.getContentElement().getAttribute('id'));

        this.__textAreaEvidence.setValue(this._oldEvidence);
        this.highlight(this.__textAreaEvidence.getContentElement().getAttribute('id'));
    },


    /**
     * show or hide animated wait logo
     */
    show_wait_animation : function(task, wait) {
        if (wait){
            this["_img_wait_" + task].show();
        } else {
            this["_img_wait_" + task].hide();
        }
    },


    /**
     * show or hide message
     */
    notify : function(message, delay, callback) {
        if (message && message != '') {
            var msg = '<div style="background-color: #bee280;"><center><h1>' + message + '</h1></center></div>';
            this._html_flashmsg.setHtml(msg);

            var fadeIN = function(val, t) {
                var fadeinInterval = setTimeout( function() {
                      if (val < 1.0) {
                        t._html_flashmsg.setOpacity(val);
                        fadeIN(val + 0.1, t);
                      } else {
                        fadeOUT(1.0, t);
                      }
                }, delay || 200);
            };

            var fadeOUT = function(val, t) {
                var fadeoutInterval = setTimeout( function() {
                      if (val > 0.0) {
                        t._html_flashmsg.setOpacity(val);
                        fadeOUT(val - 0.1, t);
                      } else {
                        t._html_flashmsg.hide();
                        callback && callback.call(t||this);
                      }
                }, delay || 200);
            };

            this._html_flashmsg.show();
            fadeIN(0, this);
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
    send_user_stats : function() {
        var currentdate = new Date();
        var date = currentdate.getDate() + "/"
                + (currentdate.getMonth()+1)  + "/"
                + currentdate.getFullYear();
        var time = currentdate.getHours() + ":"
                + currentdate.getMinutes() + ":"
                + currentdate.getSeconds();
        // var url = 'http://jsonip.appspot.com?callback=?';
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
    highlight : function(id) {
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
    change_visibility : function(e) {

        if (this.__var_show_ctr_infsettings) {
            this._ctr_infsettings.show();
        } else {
            this._ctr_infsettings.exclude();
            this._ctr_graph_visualization.show();
        }
    },


    /**
     * initialize action cores selectbox in browser page
     */
    init : function() {

        var req = new qx.io.request.Xhr("/prac/_init", "GET");
        req.addListener("success", function(e) {
            var tar = e.getTarget();
            var response = tar.getResponse();

            this.__wordnetconcepts = response.data;

            // set examples for inference and learning
            for (var i = 0; i < response.actioncores.length; i++) {
                 this._sel_ac.add(new qx.ui.form
                                              .ListItem(response.actioncores[i]));
            }
        }, this);
        req.send();
    },


    /**
    * Update fields when changing the example folder for inference
    */
    change_ac : function(e){
        this.__selected_ac = this._sel_ac.getSelection()[0].getLabel();
        var req = new qx.io.request.Xhr("/prac/_change_ac", "POST");
        req.setRequestHeader("Content-Type", "application/json");
        req.setRequestData({"ac": this.__selected_ac});
        req.addListener("success", function(e) {
            var tar = e.getTarget();
            var response = tar.getResponse();
            this.__roles = response.roles;

            this._ctr_rolesconcepts.removeAll();
            for (var role = 0; role < response.roles.length; role++) {
                this['model_' + response.roles[role]] = new combotable
                                                            .SearchableModel();
                this['model_' + response.roles[role]]
                                            .setColumns(['Id','Data'],
                                                        ['id','data']);
                this['model_' + response.roles[role]].setData(this.__wordnetconcepts);
                this['select_' + response.roles[role]] = new combotable
                                    .ComboTable(this['model_' +
                                                     response.roles[role]])
                                    .set({
                                        width: 200,
                                        placeholder: 'Select a WordNet concept',
                                        value: '',
                                        model: '0'
                                    });
                this._ctr_rolesconcepts.add(new qx.ui.basic
                                             .Label('<b>' +
                                                    response.roles[role] +
                                                    ': </b>')
                                             .set({rich:true}), {row: role,
                                                                 column: 0});
                this._ctr_rolesconcepts.add(this['select_' +
                                         response.roles[role]], {row: role,
                                                                 column: 1});
            }
        }, this);
        req.send();
    },


    /**
    * Update fields when changing the example folder for inference
    */
    get_dists : function(e){
        this.show_wait_animation('br', true);
        var roles_selects = this._ctr_rolesconcepts.getChildren();
        this.__success_distributions = false;
        this._sel_role.removeAll();

        var roles = {};
        for (var r_sel = 0; r_sel < roles_selects.length; r_sel+=2) {
            var role = roles_selects[r_sel].getValue()
                                           .replace('<b>','')
                                           .replace(': </b>','');
            var v = roles_selects[r_sel+1].getSelectedRowData();
            roles[role] = v ? v.value : null;
        }

        var req = new qx.io.request.Xhr("/prac/_change_distr", "POST");
        req.setRequestHeader("Content-Type", "application/json");
        req.setRequestData({"ac": this.__selected_ac, "roles": roles});
        req.addListener("success", function(e) {
            var tar = e.getTarget();
            var response = tar.getResponse();
            this.show_wait_animation('br', false);
            this.__distributions = response.distributions;
            this.__success_distributions = response.success;

            if (response.success) {
                roles = Object.keys(response.distributions);

                for (var role = 0; role < roles.length; role++) {
                    this._sel_role.add(new qx.ui.form.ListItem(roles[role]));
                }
            } else {
                this.notify(response.msg, 300);
            }

        }, this);
        req.send();
    },


    /**
    * Update distribution svg according to selected role
    */
    change_distr : function(e){
        var selected_role = this._sel_role.getSelection();
        var parentwidth = this._scrollctr_html_distr.getWidth();
        if (selected_role.length == 0) {
            this.__html_distr.setHtml('');
        } else {
            this.__html_distr.setHtml(this.__distributions[selected_role[0]
                                                            .getLabel()]);
        }
        this.__html_distr.setMinWidth(parentwidth);
        this.__html_distr.setMaxWidth(parentwidth);
    },


    /**
    * Zoom in and out of distribution svg
    */
    dist_zoom : function(e){
        var pos = e.getData();
        var parentwidth = this._scrollctr_html_distr.getWidth();
        var parentheight = this._scrollctr_html_distr.getHeight();
        var newwidth = pos / 100 * parentwidth;
        var newheight = pos / 100 * parentheight;

        this.__html_distr.setMinWidth(newwidth);
        this.__html_distr.setMaxWidth(newwidth);
        this.__html_distr.setMinHeight(newheight);
        this.__html_distr.setMaxHeight(newheight);
        this._grp_slider.value.setValue('Zoom: ' + Math.round(newwidth/parentwidth * 100).toString() + '%');
    }
  }
});
