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
    this._height = 900;
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

    node = this._svg.selectAll(".node");
    link = this._svg.selectAll(".link");

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
	var node = this._layer.selectAll(".node");
	var link = this._layer.selectAll(".link");
		
	node = node.data(this._nodes)
	  .enter().append("g")
      .attr("class", "node")
    
    node.append("rect")//.transition().duration(500).attr('width', 150)
      	.style("stroke", "#000")
      	.style("stroke-width", .1)
      	.style("fill", function(d) { return "#eeeeee"; })
      	.attr("width", function(d) { return 100;})
      	.attr("height", 20)
		.attr("transform", "translate(-50, -13)")
    	.call(this._force.drag);
//	node.append("circle")
//		.attr("r", 5);
	
    node.append("text")
      	.style("stroke", function(d) { return "#000"; })
		.text(function(d) {return d.name})
//		.call(this._force.drag)
		.style("stroke-width", .2)
	    .style({"font-family":"Arial, Helvetica, sans-serif","font-size":"12px","z-index":"999999999"})
	    .style("text-anchor", "middle");
//		.attr("class", "node")
//		.attr("width", 100)
//		.attr("height", 50)
//		.style("fill", function(d) { return "#DC143C"; })
//		.style("stroke-width", function(d) { return "1.5px"; })
//		.call(this._force.drag)
//		.insert("text").text("hello")
	
	link = link.data(this._links)
	  .enter().append("line")
		.attr("class", "link")
		.style("stroke-width", function(d) { return Math.sqrt(d.value); })
		.style("stroke", function(d) { return "#999"; })
		.style("stroke-opacity", function(d) { return ".6"; });
	
	node = this._layer.selectAll(".node");
	link = this._layer.selectAll(".link");
	  
	this._force.on("tick", function() {
		link.attr("x1", function(d) { return d.source.x; })
			.attr("y1", function(d) { return d.source.y; })
			.attr("x2", function(d) { return d.target.x; })
			.attr("y2", function(d) { return d.target.y; });
//		node.attr("dx", function(d) { return d.x; })
//		    .attr("dy", function(d) { return d.y; });
		node.attr("transform", function(d) { return "translate(" + d.x + "," + d.y + ")"; });
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
      this._svg.append( "svg:g" ).attr( "class", name );
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
	  this._force.charge(charge);
  }
};

// TYPE HANDLER

rap.registerTypeHandler( "d3graph.ForceDirectedGraph", {
  factory: function( properties ) {
    var parent = rap.getObject( properties.parent );
    return new d3graph.ForceDirectedGraph( parent );
  },
  
  destructor: "destroy",
 
  methods: ["render", "addNode", "addLink", "restart", "setCharge"],
  
  properties: [],

  events: [ "Selection" ]
  
} );
