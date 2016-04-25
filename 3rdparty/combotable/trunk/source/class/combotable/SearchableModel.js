/* ************************************************************************

   Copyright:
     Tobias Oetiker, OETIKER+PARTNER AG, www.oetiker.ch
     Mustafa Sak, SAK systems, www.saksys.de
     
   License:
     LGPL: http://www.gnu.org/licenses/lgpl.html
     EPL: http://www.eclipse.org/org/documents/epl-v10.php
     See the LICENSE file in the project's top-level directory for details.

   Authors:   
     * Tobias Oetiker (oetiker)
     * Mustafa Sak

************************************************************************ */

/**
 * Add searchString property to the Simple model. The searching will always happen on the second column.
 */
qx.Class.define("combotable.SearchableModel", {
    extend : qx.ui.table.model.Simple,

    properties : {
        /**
         * The string to search for in the second column of the dataset.
         */
        searchString : {
            init     : null,
            apply    : '_applySearchString',
            nullable : true,
            check    : "String"
        },


        /**
         * Only search from the start of the string
         */
        anchorFront : {
            init  : false,
            check : "Boolean"
        },


        /**
         * Take string as a regular expression
         */
        searchAsRegEx : {
            init  : false,
            check : "Boolean"
        }
    },

    members : {
        __allRows : null,


        /**
         * Overridden from {@link qx.ui.table.model.Simple} with an additional
         * argument tucked at the end to tell the model if the input is to be
         * stored for filtering or is the output of the filtering operation.
         */
        setData : function(rowArr, clearSorting, filtered) {
            if (!filtered) {
                this.__allRows = rowArr;
            }

            this.base(arguments, rowArr, clearSorting);
        },

        /**
         * Overridden from {@link qx.ui.table.model.Simple}.
         */
        addRows : function(rowArr, startIndex, clearSorting) {
            this.setData(this.__allRows);
            this.base(arguments, rowArr, startIndex, clearSorting);
            this._applySearchString(this.getSearchString());
        },

        /**
         * Overridden from {@link qx.ui.table.model.Simple}.
         */
        setRows : function(rowArr, startIndex, clearSorting) {
            this.setData(this.__allRows);
            this.base(arguments, rowArr, startIndex, clearSorting);
            this._applySearchString(this.getSearchString());
        },



        /**
         * Overridden from {@link qx.ui.table.model.Simple}.
         */
        removeRows : function(startIndex, howMany, clearSorting) {
            this.setData(this.__allRows);
            this.base(arguments, startIndex, howMany, clearSorting);
            this._applySearchString(this.getSearchString());
        },


        /**
         * As the search string is changed, re-filter the content of the table
         *
         * @param newString {var} old string
         * @param oldString {var} new string
         * @return {void} 
         */
        _applySearchString : function(newString, oldString) {
            if (oldString == newString) {
                return;
            }

            var data = [];
            var d = this.__allRows;
            var anchor = this.getAnchorFront();

            if (this.getSearchAsRegEx()) {
                var rx = new RegExp(newString,"i");

                for (var i=0; i<d.length; i++) {
                    var idx = d[i][1].search(rx);

                    if (anchor ? idx == 0 : idx != -1) {
                        data.push(d[i]);
                    }
                }
            }
            else {
                for (var i=0; i<d.length; i++) {
                    var idx = d[i][1].indexOf(newString);

                    if (anchor ? idx == 0 : idx != -1) {
                        data.push(d[i]);
                    }
                }
            }
            this.setData(data, true, true);
        }
    }
});
