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
qx.Class.define("pracweb.Graph",
{
  extend : qx.ui.core.Widget,

  /*
  *****************************************************************************
     MEMBERS
  *****************************************************************************
  */
  construct: function(prarent) {
    this.container = new qxd3.Svg("#viz", 'pracgraph');
    this.d3 = this.container.getD3();
    this.WAITMSEC = 500;

    this.w = window.innerWidth;
    this.h = window.innerHeight;

    this.svnContainer = this.d3.select('#viz').select("svg").select("g");
      // .append("svg:svg")
      // .attr("width", "100%")
      // .attr("height", "100%")
      // .attr("id","svg")
      // .append('svg:g');


    this.svnContainer.append("defs").selectAll("marker")
      .data(["dashedred", "strokegreen", "dashed", "strokeblue", "arrowhead", "default"])
      .enter().append("marker")
      .attr("id", function(d) { return d; })
      .attr("viewBox", "0 -5 10 10")
      .attr("refX", 15)//15
      .attr("refY", -1.5)
      .attr("markerWidth", 6)
      .attr("markerHeight", 6)
      .attr("orient", "auto")
      .append("path")
      .attr("d", "M0,-5L10,0L0,5 Z");



    this.force = this.d3.layout.force();
    this.nodes = this.force.nodes();
    this.links = this.force.links();

    // Make it all go
    this.update();
  },

  members :
  {

    addNode : function (id) {
      this.nodes.push({"id":id});
      this.update();
    },


    updateData : function (data) {
      var tmpNodes = [];
      var tmpLinks = [];
      var nodesCopy = [];
      var linksCopy = [];

      // create temporary lists of nodes and links containing only new entries
      for (var a in data) {
        if (!this.nodeInList(data[a].source, tmpNodes) && typeof this.findNodeInList(data[a].source, tmpNodes) === "undefined") {
          tmpNodes.push(data[a].source);
        }
        if (!this.nodeInList(data[a].target, tmpNodes) && typeof this.findNodeInList(data[a].target, tmpNodes) === "undefined") {
          tmpNodes.push(data[a].target);
        }
        tmpLinks.push({"source": data[a].source,"target": data[a].target,"value": data[a].value, "arcStyle":data[a].arcStyle});
      }

      // add all nodes and links
      var addnodesandlinks = function(dataIndex, t) {
          var aInterval = setTimeout( function() {
          if (typeof t.findNode(data[dataIndex].source) === "undefined") {
            t.nodes.push({"id":data[dataIndex].source});
          }
          if (typeof t.findNode(data[dataIndex].target) === "undefined") {
            t.nodes.push({"id":data[dataIndex].target});
          }
          t.update();
          //check if there is already a link between source and target node
          var index = t.findLinkIndex(t.findNode(data[dataIndex].source), t.findNode(data[dataIndex].target), data[dataIndex].value, t.links);
          if (index == -1) {
            // if not, create it
            t.links.push({"source": t.findNode(data[dataIndex].source),"target": t.findNode(data[dataIndex].target),"value": [data[dataIndex].value], "arcStyle":data[dataIndex].arcStyle});
          } else {
            // if so, check if the old value contains the new one
            if (t.links[index].value.indexOf(data[dataIndex].value) == -1) {
              // if not, update still existing links
              var newVal = t.findLinkIndexWithValue(t.links[index].source, t.links[index].target, t.links[index].value, tmpLinks);
              // replace old non existent values and add new one
              t.links.splice(index,1);
              t.update();
              t.links.push({"source": t.findNode(data[dataIndex].source),"target": t.findNode(data[dataIndex].target),"value": newVal, "arcStyle":data[dataIndex].arcStyle});
            }
          }

          t.update();
          dataIndex++;
          if (dataIndex >= data.length) {
            nodesCopy = t.nodes.slice();
            removenodes(0, t);
          } else {
            addnodesandlinks(dataIndex, t);
          }
        }, t.WAITMSEC);
      };


      // remove all nodes and links, that are not in the new data
      var removenodes = function(dIndex, t) {
        var rInterval = setTimeout( function() {
          var n = nodesCopy[dIndex].id;
          
          if (!t.nodeInList(nodesCopy[dIndex].id, tmpNodes)) {
            var i = 0;
            while (i < t.links.length) {
              if ((t.links[i]['source'].id == n)||(t.links[i]['target'].id == n))
              {
                t.links.splice(i,1);
                t.update();
              }
              else i++;
            }
            t.nodes.splice(t.findNodeIndex(nodesCopy[dIndex].id),1);
            t.update();
          }
          t.update();
          dIndex++;
          if (dIndex < nodesCopy.length) {
            removenodes(dIndex, t);
          } else {
            linksCopy = t.links.slice();
            console.log('Finished updating graph');
          }
        }, t.WAITMSEC);
      };

      addnodesandlinks(0, this);
      this.update();
    },


    findLinkIndex : function(source, target) {
      for (var i=0; i < this.links.length; i++) {
        if (this.links[i].source.id == source.id && this.links[i].target.id == target.id) {
          return i;
        }
      }
      return -1; // if link does not exist in l
    },

    // returns a list of indices that need to be removed from values
    findLinkIndexWithValue : function(source, target, value, l) {
      var rem = [];
      for (var i=0; i < l.length; i++) {
        if (l[i].source == source.id && l[i].target == target.id) {
          for (var oldVal = 0; oldVal < value.length; oldVal++) {
            rem.push(l[i].value);
          }
        }
      }
      return rem;
    },

    nodeInList : function(id, n) {
      for (var no = 0; no < n.length; no++) {
        if (n[no] == id) {
          return true;
        }
      }
      return false;
    },

    linkInList : function(id, l) {
      for (var no = 0; no < l.length; no++) {
        if (l[no].source == id.source && l[no].target == id.target && l[no].value == id.value ) {
          return true;
        }
      }
      return false;
    },

    findNodeInList : function(id, n) {
      for (var i in n) {
        if (i.id === id) return i;};
    },


    removeNode : function (id) {
      var i = 0;
      var n = this.findNode(id);
      while (i < this.links.length) {
        if ((this.links[i]['source'] == n)||(this.links[i]['target'] == n))
        {
          this.links.splice(i,1);
        }
        else i++;
      }
      this.nodes.splice(this.findNodeIndex(id),1);
    },

    removeLink : function (source,target){
      for(var i=0; i < this.links.length; i++)
      {
        if(this.links[i].source.id == source && this.links[i].target.id == target)
        {
          this.links.splice(i,1);
          break;
        }
      }
      this.update();
    },

    removeallLinks : function(){
      this.links.splice(0,this.links.length);
      this.update();
    },

    removeAllNodes : function(){
      this.nodes.splice(0,this.nodes.length);
      this.update();
    },

    clear : function() {
      this.removeallLinks();
      this.removeAllNodes();
    },

    addLink : function (source, target, value, arcStyle) {
      this.links.push({"source": this.findNode(source),"target": this.findNode(target),"value": value,"arcStyle": arcStyle});
      this.update();
    },

    showlinks : function() {
      for (var l = 0; l < this.links.length; l++) {
        console.log(this.links[l].source.id + " " + this.links[l].target.id + " " + this.links[l].value + " " + this.links[l].arcStyle);
      }
    },

    showNodes : function() {
      var str = '';
      for (var l = 0; l < this.nodes.length; l++) {
        str += this.nodes[l].id;
        str += ", ";
      }
      console.log(str);
    },

    findNode : function(id) {
      for (var i in this.nodes) {
        if (this.nodes[i].id === id) return this.nodes[i];};
    },

    findNodeIndex : function(id) {
      for (var i=0; i < this.nodes.length; i++) {
        if (this.nodes[i].id == id){
          return i;
        }
      };
    },

    update : function () {

      var path = this.svnContainer.selectAll("path.link")
        .data(this.links, function(d) {
                return d.source.id + "-" + d.target.id; 
                });

      var pathEnter = path.enter().append("path")
        .attr("id", function(d) { return d.source.id + "-" + d.target.id; })
        .attr("class", function(d) { return "link " + d.arcStyle; })
        .attr("marker-end", function(d) { return "url(#" + d.arcStyle + ")"; });

      path.exit().remove();

      var edgelabels = this.svnContainer.selectAll(".label")
            .data(this.links, function(d) {
                return d.source.id + "-" + d.target.id; 
                });

      var edgelabelsEnter = edgelabels.enter().append('text')
          .style("pointer-events", "none")
          .attr('class', 'label')
          .text(function(d){ return d.value.join(' / '); });

      edgelabels.exit().remove();

      var circle = this.svnContainer.selectAll("g.node")
        .data(this.nodes, function(d) { return d.id; } );

      var circleEnter = circle.enter().append("g")
        .attr("class", "node")
        .call(this.force.drag);

      circleEnter.append("svg:circle")
        .attr("r", 10)
        .attr("id", function(d) { return d.id; } );

      // circleEnter.append("svg:rect")
      //   .attr("width", function(d) {return 10*d.id.length;}) // set width according to text length
      //   .attr("height", 25)
      //   .attr("id", function(d) { return d.id; } );

      circleEnter.append("svg:text")
        .attr("class","textClass")
        .attr("dx", function (d) { return 5; }) // move inside rect
        .attr("dy", function (d) { return 15; }) // move inside rect
        .text( function(d) { return d.id; } );

      circle.exit().remove();


      var tick = function () {
        path.attr("d", linkArc);
        edgelabels.attr('d', linkArc);
        edgelabels.attr('transform', rotateLabel);
        edgelabels.attr('x', transformLabelX);       
        edgelabels.attr('y', transformLabelY);       
        circle.attr("transform", transform);
      };

      var rotateLabel = function (d) {
        var bbox = this.getBBox();
        var rx = bbox.x+bbox.width/2;
        var ry = bbox.y+bbox.height/2;
        var dX = d.target.x - d.source.x;
        var dY = d.target.y - d.source.y;
        var rad = Math.atan2(dX, dY);
        var deg = -90-rad * (180 / Math.PI);
        return 'rotate(' + deg +' '+rx+' '+ry+')';
      };


      var linkArc = function (d) {
        var dx = d.target.x - d.source.x,
            dy = d.target.y - d.source.y,
            dr = Math.sqrt(dx * dx + dy * dy);
        return "M" + 
            d.source.x + "," + 
            d.source.y + "A" + 
            dr + "," + dr + " 0 0,0 " + 
            d.target.x + "," + 
            d.target.y;
      };

      // move arc label to arc
      var calcLabelPos = function (d, bbox) {
        var scale = 0.4; // distance from arc
        var origPos = { x: (d.source.x + d.target.x ) /2 - bbox.width/2, y: (d.source.y + d.target.y) /2 }; // exact middle between source and target
        var dir = { x: d.target.x - d.source.x, y: d.target.y - d.source.y }; // direction source -> target
        var rot = { x: dir.y, y: -dir.x }; // rotate direction -90 degrees
        var ltemp = Math.sqrt(rot.x * rot.x + rot.y * rot.y) / 100 // normalize length
        var length = ltemp !== 0 ? ltemp : 0.1; // if length is 0, set to small value to prevent NaN
        var rotNorm = { x: rot.x / length, y: rot.y / length }; // normalize rotation direction
        return { x: origPos.x - scale * rotNorm.x, y: origPos.y - scale * rotNorm.y};// return moved position
      };

      var transform = function (d) {
        return "translate(" + d.x + "," + d.y + ")";
      };

      var transformLabel = function (d) {
        return "translate(" + d.source.x + "," + d.source.y + ")";
      };

      var transformLabelX = function (d) {
        var bbox = this.getBBox();
        return calcLabelPos(d, bbox).x;
      };

      var transformLabelY = function (d) {
        var bbox = this.getBBox();
        return calcLabelPos(d, bbox).y;
      };

      this.force
        .size([.8*window.innerWidth, .8*window.innerHeight])
        .linkDistance( this.h/2 )
        .charge(-1000)
        .on("tick", tick)
        .gravity( .025 )
        .distance( 250 )
        .start();
    }
  }
});
