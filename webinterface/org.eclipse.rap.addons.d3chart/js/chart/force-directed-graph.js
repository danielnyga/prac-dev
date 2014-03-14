/*******************************************************************************
 * Copyright (c) 2013 EclipseSource and others.
 * All rights reserved. This program and the accompanying materials
 * are made available under the terms of the Eclipse Public License v1.0
 * which accompanies this distribution, and is available at
 * http://www.eclipse.org/legal/epl-v10.html
 *
 * Contributors:
 *    Ralf Sternberg - initial API and implementation
 ******************************************************************************/

d3graph = {};

d3graph.ForceDirectedGraph = function( parent ) {
	this._nodes = []
	this._links = []
	this._width = 960,
    this._height = 500;
    this._color = d3.scale.category20();	
    
    this._renderer = this;
    this._element = this.createElement( parent );
    this._padding = 20;

    this._needsLayout = true;
    var that = this;
    
    this._force = d3.layout.force()
	    .charge(-120)
	    .linkDistance(30)
	    .size([this._width, this._height]);
    
    this._svg = d3.select(this._element).append("svg")
    .attr("width", this._width)
    .attr("height", this._height)
    .attr( "class", "chart");
	this._layer = this.getLayer("layer");    
    
    
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
    

};

d3graph.ForceDirectedGraph.prototype = {

  sayHello: function() {
	  alert("hello");
  },
  
  addNode: function( node ) {
	  this._nodes.push(node);
  },
  
  addLink: function( link ) {
	  this._links.push(link);
  },
  
  showGraph: function () {
	  this._force
      .nodes(this._nodes)
      .links(this._links)
      .start();

	  var node = this._layer.selectAll(".node")
	      .data(this._nodes)
	    .enter().append("circle")
	      .attr("class", "node")
	      .attr("r", 5)
	      .style("fill", function(d) { return "#DC143C"; })
	      .style("stroke", function(d) { return "#fff"; })
	      .style("stroke-width", function(d) { return "1.5px"; })
	      .call(this._force.drag);
	  
	  var link = this._layer.selectAll(".link")
	  	 	.data(this._links)
	  	.enter().append("line")
	  		.attr("class", "link")
	  		.style("stroke-width", function(d) { return Math.sqrt(d.value); })
	  		.style("stroke", function(d) { return "#999"; })
	  		.style("stroke-opacity", function(d) { return ".6"; });

	  node.append("title")
	  	.text(function(d) { return d.name; });
  
	  this._force.on("tick", function() {
		    link.attr("x1", function(d) { return d.source.x; })
		        .attr("y1", function(d) { return d.source.y; })
		        .attr("x2", function(d) { return d.target.x; })
		        .attr("y2", function(d) { return d.target.y; });
	
		    node.attr("cx", function(d) { return d.x; })
		        .attr("cy", function(d) { return d.y; });
		  });
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
      this._svg.append( "svg:g" ).attr( "class", name );
      layer = this._svg.select( "g." + name );
    }
    return layer;
  },

  _resize: function( clientArea ) {
    this._width = clientArea[ 2 ];
    this._height = clientArea[ 3 ];
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
	  this.showGraph();
  }
};

// TYPE HANDLER

rap.registerTypeHandler( "d3graph.ForceDirectedGraph", {
  factory: function( properties ) {
    var parent = rap.getObject( properties.parent );
    return new d3graph.ForceDirectedGraph( parent );
  },
  
  destructor: "destroy",
 
  methods: ["showGraph", "render", "sayHello", "addNode", "addLink"],
  
  properties: [],

  events: [ "Selection" ]
  
} );
