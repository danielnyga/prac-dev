
d3graph = {};

d3graph.ForceDirectedGraph = function( parent ) {
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
	  graph = {
		  "nodes":[
		           {"name":"Myriel","group":1}  ],
		  "links": []
	  };
	  
	  this._force
	      .nodes(graph.nodes)
	      .links(graph.links)
	      .start();
	  
	  var node = this._layer.selectAll(".node")
	      .data(graph.nodes)
	    .enter().append("circle")
	      .attr("class", "node")
	      .attr("r", 5)
	      .style("fill", "#000000" )
	      .call(this._force.drag);
	  
	  this._force.on("tick", function() {
//  	    link.attr("x1", function(d) { return d.source.x; })
//  	        .attr("y1", function(d) { return d.source.y; })
//  	        .attr("x2", function(d) { return d.target.x; })
//  	        .attr("y2", function(d) { return d.target.y; });

  	    node.attr("cx", function(d) { return d.x; })
  	        .attr("cy", function(d) { return d.y; });
  	  });
	  d3.json("miserables.json", function(error, graph) {
		  alert("processing " + graph.nodes);
    	  this._force
    	      .nodes(graph.nodes)
    	      .links(graph.links)
    	      .start();
    	  var link = this._layer.selectAll(".link")
    	      .data(graph.links)
    	    .enter().append("line")
    	      .attr("class", "link")
    	      .style("stroke-width", function(d) { return Math.sqrt(d.value); });

    	  var node = this._layer.selectAll(".node")
    	      .data(graph.nodes)
    	    .enter().append("circle")
    	      .attr("class", "node")
    	      .attr("r", 5)
    	      .style("fill", "#000000" )
    	      .call(force.drag);

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
    });
  }
//	  alert("render")
//	  chart._svg.append( "svg:rect" )
//      	.attr( "x", 0)
//      	.attr( "y", 0 )
//      	.attr( "width", "100%" )
//      	.attr( "height", "100%")
//      	.attr("fill", "#000000")
//	  }
};

//d3graph.ForceDirectedGraph.prototype = {
//
//  destroy: function() {
//    this._chart.destroy();
//  }
//}
//
//
//// TYPE HANDLER
//

rap.registerTypeHandler( "d3graph.ForceDirectedGraph", {
  factory: function( properties ) {
    var parent = rap.getObject( properties.parent );
    return new d3graph.ForceDirectedGraph( parent );
  },
  
  destructor: "destroy",
 
  properties: [],

  events: [ "Selection" ]
  
} );
