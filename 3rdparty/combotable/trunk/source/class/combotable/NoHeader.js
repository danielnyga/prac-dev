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
 * A Non-Header-Header
 */
qx.Class.define("combotable.NoHeader", {
    extend : qx.ui.table.pane.Header,

    construct : function(paneScroller) {
        this.base(arguments, paneScroller);

        this.__privateScroller = paneScroller;
    },

    members : {
        __privateScroller : null,

        /**
         * Overridden from {@link qx.ui.core.Widget.getContainerLocation} so that it
         * works with the header disabled.
         */
        getContainerLocation : function(mode) {
            var domEl = this.getContainerElement().getDomElement();

            if (domEl) {
                return qx.bom.element.Location.get(domEl, mode);
            }
            else {
                domEl = this.__privateScroller.getContainerElement().getDomElement();

                if (domEl) {
                    var loc = qx.bom.element.Location.get(domEl, mode);
                    loc.bottom = loc.top;
                    return loc;
                }
                else {
                    return {
                        left   : 0,
                        right  : 0,
                        top    : 0,
                        bottom : 0
                    };
                }
            }
        }
    }
});
