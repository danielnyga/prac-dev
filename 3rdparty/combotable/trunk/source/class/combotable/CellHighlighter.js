/* ************************************************************************

   Copyright:
     Tobias Oetiker, OETIKER+PARTNER AG, www.oetiker.ch
     
   License:
     LGPL: http://www.gnu.org/licenses/lgpl.html
     EPL: http://www.eclipse.org/org/documents/epl-v10.php
     See the LICENSE file in the project's top-level directory for details.

   Authors:   
     * Tobias Oetiker (oetiker)

************************************************************************ */

/**
 * Highlight part of the cell content.
 */
qx.Class.define("combotable.CellHighlighter", {
    extend : qx.ui.table.cellrenderer.Default,

    properties : {
        /**
         * Set the highlite string.
         */
        searchString : {
            init  : null,
            check : "String",
            apply : "_applySearchString",
            nullable: true
        }
    },

    members : {
        __searchRx : null,
        __searchEsc : null,


        /**
         * populate private variables to accellerate highliting operation.
         *
         * @param value {var} new value
         * @param old {var} old value
         * @return {void} 
         */
        _applySearchString : function(value, old) {
            if (value == null){
                this.__searchRx = null;
                this.__searchEsc = null;
            }
            else {
                this.__searchRx = new RegExp(this._preg_quote(value), 'ig');
                this.__searchEsc = qx.bom.String.escape(value);
            }
        },

        /**
         * Just a simple qooxdoo table celll 
         *
         * @param cellInfo {var} disregarded ...
         * @return {string} the class of the cell
         */
        _getCellClass : function(cellInfo) {
            return "qooxdoo-table-cell";
        },

        /**
         * Quote a string.
         *
         * With inspiration from {@link http://stackoverflow.com/questions/280793/case-insensitive-string-replacement-in-javascript}
         *
         * @param str {String} input
         * @return {String} regexp quoted 
         */
        _preg_quote : function(str) {
            return String(str).replace(/([\\\.\+\*\?\[\^\]\$\(\)\{\}\=\!\<\>\|\:])/g, "\\$1");
        },


        /**
         * Do the actual highlighting
         *
         * @param cellInfo {var} the cell content
         * @return {String} the html of the cell
         */
        _getContentHtml : function(cellInfo) {
            var str = this._formatValue(cellInfo);

            if (this.__searchRx) {
                return str.replace(this.__searchRx, '<strong>' + this.__searchEsc + '</strong>');
            } else {
                return str;
            }
        }
    }
});
