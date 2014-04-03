/*******************************************************************************
 * Copyright (c) 2013 EclipseSource and others. All rights reserved. This
 * program and the accompanying materials are made available under the terms of
 * the Eclipse Public License v1.0 which accompanies this distribution, and is
 * available at http://www.eclipse.org/legal/epl-v10.html
 * 
 * Contributors: Ralf Sternberg - initial API and implementation
 ******************************************************************************/

d3graph = {};
var that;

d3graph.ForceDirectedGraph = function( parent ) {
    this._width = 900;
    this._height = 700;
    this._color = d3.scale.category20();	
    
    this._renderer = this;
    this._element = this.createElement( parent );
    this._padding = 20;

    this._needsLayout = true;
    that = this;
    
    this._force = d3.layout.force()
	    .charge(-200)
	    .linkDistance(30)
	    .size([this._width, this._height]);
   		
    this._nodes = this._force.nodes()
    this._links = this._force.links()
    
    this._svg = d3.select(this._element).append("svg")
    	.attr("width", this._width)
    	.attr("height", this._height)
    	.attr("class", "chart");
    
    this._svg.append('defs').append('marker')
    .attr({
      id: 'arrowhead',
      refX: 13,
      refY: 4,
      markerUnits: 'strokeWidth',
      markerWidth: 10,
      markerHeight: 8,
      orient: 'auto'
    }).append('path').attr({
      d: 'M 0 0 L 10 4 L 0 8 Z',
      fill: 'black'
    });
    


//    var linkG = this._svg.append("g")
//    var nodeG = this._svg.append("g")
//    this._layer = this.getLayer("layer");    
    this.link_layer = this.getLayer("links")
    this.node_layer = this.getLayer("nodes")
    this.label_layer = this.getLayer("labels")
    
    link = this.link_layer.selectAll(".link")
    		.attr('marker-end', 'url(#arrowhead)');
    node = this.node_layer.selectAll(".node")

	
    rap.on( "render", function() {
      if( that._needsRender ) {
        if( that._needsLayout ) {
          that._renderer.initialize( that );
          that._needsLayout = false;
        }
        that._renderer.render( that );
        that._needsRender = false;
      }
    } );
    
    parent.addListener( "Resize", function() {
      that._resize( parent.getClientArea() );
    } );
    
    this._resize( parent.getClientArea() );
//    this.restart();
};

d3graph.ForceDirectedGraph.prototype = {
		
  addNode: function( node ) {
	this._nodes.push(node);
	this.restart();
  },
  
  addLink: function( link ) {
	this._links.push(link);
	this.restart();
  },
  
  restart: function () {
	var link = this.link_layer.selectAll("line.link")
				.attr('marker-end', 'url(#arrowhead)');
		
	var node = this.node_layer.selectAll(".node");

	var label = this.label_layer.selectAll("text.label");
	
	node.append("title")
      .text(function(d) { return d.name; });
	
	link = link.data(this._links)
	  	.enter()
	  	.append("line")
		.attr("class", "link")
		.attr('marker-end', 'url(#arrowhead)')
		.style("stroke-width", function(d) { return 2.; })
		.style("stroke", function(d) { return "black"; })
		.style("stroke-opacity", function(d) { return Math.max(0.2, d.strength); })
		.style({"z-index":"0"});
		
	link.insert("text")
		.style("stroke", function(d) { return "#000"; })
		.text(function(d) {return d.label})
		.style("stroke-width", .2)
	    .style({"font-family":"Arial, Helvetica, sans-serif","font-size":"12px"}) //,"z-index":"999999999"
	    .style("text-anchor", "middle");
	
	label = label.data(this._links)
	  .enter().append('text')
	  .attr("class", "label")
	    .attr("x", function(d) { return (d.source.y + d.target.y) / 2; }) 
	    .attr("y", function(d) { return (d.source.x + d.target.x) / 2; }) 
	    .attr("text-anchor", "middle") 
	    .text(function(d) {return d.label;});  

	g = node.data(this._nodes)
	  .enter().insert("g")
      .attr("class", "node")
//	  .style({"z-index": "99999"})
    
	g.insert("rect")//.transition().duration(500).attr('width', 150)
//		.attr("class", "node")
      	.style("stroke", "#000")
      	.style("stroke-width", .2)
      	.style({"z-index":"999999"})
      	.style("fill", function(d) { return d.color; })
      	.attr("width", function(d) { return 100;})
      	.attr("height", 20)
		.attr("transform", "translate(-50, -13)")
    	.call(this._force.drag)
//		.enter()
//		.append("circle")
//		.attr("class", "node")
//		.attr("r", 5)
//		.style("fill", "green")
//		.style("stroke", "white");
	
    g.insert("text")
      	.style("stroke", function(d) { return "#000"; })
		.text(function(d) {return d.label})
		.call(this._force.drag)
		.style("stroke-width", .2)
	    .style({"font-family":"Arial, Helvetica, sans-serif","font-size":"12px"}) //,"z-index":"999999999"
	    .style("text-anchor", "middle");

    
 //		.attr("class", "node")
//		.attr("width", 100)
//		.attr("height", 50)
//		.style("fill", function(d) { return "#DC143C"; })
//		.style("stroke-width", function(d) { return "1.5px"; })
//		.call(this._force.drag)
//		.insert("text").text("hello")
	
	link = this.link_layer.selectAll(".link");
	node = this.node_layer.selectAll(".node");
	label = this.label_layer.selectAll(".label");

	  
	this._force.on("tick", function() {
		link.attr("x1", function(d) { return d.source.x; })
			.attr("y1", function(d) { return d.source.y; })
			.attr("x2", function(d) { return d.target.x; })
			.attr("y2", function(d) { return d.target.y; });
//		link.attr("x", function(d) { return Math.min(d.source.x, d.target.x); })
//		.attr("y", function(d) { return Math.min(d.source.y, d.target.y); })
//		.attr("width", function(d) { return Math.max(d.source.x, d.target.x) - Math.min(d.source.x, d.target.x); })
//		.attr("height", function(d) { return Math.max(d.source.y, d.target.y) - Math.min(d.source.y, d.target.y); });
//		link.style({"z-index": "0"})
		node.attr("transform", function(d) { return "translate(" + d.x + "," + d.y + ")"; });
//		node.style({"z-index": "999"})
//		node.attr("cx", function(d) { return d.x; })
//		    .attr("cy", function(d) { return d.y; });
		
//		labels = link.data(this._links)
//	  	.enter().append('text')
//	    .attr("x", function(d) { return (d.source.y + d.target.y) / 2; }) 
//	    .attr("y", function(d) { return (d.source.x + d.target.x) / 2; }) 
//	    .attr("text-anchor", "middle") 
//	    .text(function(d) {return 2;}); 
	
		label.attr("x", function(d) { return (d.source.x + d.target.x) / 2; }) 
        	.attr("y", function(d) { return (d.source.y + d.target.y) / 2; }) 
	});
	
	this._force.start();
  },
  
  createElement: function( parent ) {
    var element = document.createElement( "div" );
    element.style.position = "absolute";
    element.style.left = "0";
    element.style.top = "0";
    element.style.width = "100%";
    element.style.height = "100%";
    parent.append( element );
    return element;
  },

  getLayer: function( name ) {
    var layer = this._svg.select( "g." + name );
    if( layer.empty() ) {
      this._svg.append( "g" ).attr( "class", name );
      layer = this._svg.select( "g." + name );
    }
    return layer;
  },

  _resize: function( clientArea ) {
    this._width = clientArea[ 2 ];
    this._height = clientArea[ 3 ];
    this._force.size([this._width, this._height]);
    this._svg.attr( "width", this._width ).attr( "height", this._height );
    this._scheduleUpdate( true );
  },

  _scheduleUpdate: function( needsLayout ) {
    if( needsLayout ) {
      this._needsLayout = true;
    }
    this._needsRender = true;
  },

  destroy: function() {
    var element = this._element;
    if( element.parentNode ) {
      element.parentNode.removeChild( element );
    }
  },
  
  initialize: function( chart ) {
	    this._chart = chart;
	    this._layer = chart.getLayer( "layer" );
  },
	  
  render: function( chart ) {
	  this.restart();
  },
  
  setCharge: function(charge) {
	  this._force.charge(charge['charge']);
	  this.restart();
  },
  
  setLinkDistance: function(params) {
	  this._force.linkDistance(params['linkdist']);
	  this.restart();
  },
  
  setFriction: function(params) {
	  this._force.friction(params['linkdist']);
	  this.restart();
  },
  
  setChargeDistance: function(params) {
	  this._force.chargeDistance(params['linkdist']);
	  this.restart();
  },
  
  setGravity: function(params) {
	  this._force.gravity(params['linkdist']);
	  this.restart();
  },
  
  removeAllNodes: function(params) {
	  this._nodes.length = 0;
	  this._links.length = 0;
	  this.restart();
  },
};

// TYPE HANDLER

rap.registerTypeHandler( "d3graph.ForceDirectedGraph", {
  factory: function( properties ) {
    var parent = rap.getObject( properties.parent );
    return new d3graph.ForceDirectedGraph( parent );
  },
  
  destructor: "destroy",
 
  methods: ["render", "addNode", "addLink", "restart", "setCharge", "setLinkDistance", "setFriction", 
            "setChargeDistance", "setGravity", "removeAllNodes"],
  
  properties: [],

  events: [ "Selection" ]
  
} );
