/* ************************************************************************

   qooxdoo - the new era of web development

   http://qooxdoo.org

   Copyright:
     2004-2009 1&1 Internet AG, Germany, http://www.1und1.de

   License:
     LGPL: http://www.gnu.org/licenses/lgpl.html
     EPL: http://www.eclipse.org/org/documents/epl-v10.php
     See the LICENSE file in the project's top-level directory for details.

   Authors:
     * Martin Wittemann (martinwittemann)
     * Fabian Jakobs (fjakobs)

************************************************************************ */

qx.Class.define("showcase.ui.PreviewList",
{
  extend : qx.ui.form.List,

  construct : function() {
    this.base(arguments, true);
  },

  properties :
  {
    appearance :
    {
      refine: true,
      init: "preview-list"
    },

    height : {
      refine: true,
      init: null
    }
  }
});