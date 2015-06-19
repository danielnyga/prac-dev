/**
 * Code Syntax Highlighter for MLNs.
 * Version 0.0.1
 * Copyright (C) 2015 Picklum, Mareike.
 * http://ai.uni-bremen.de/team/mareike_picklum
 * 
 * This library is free software; you can redistribute it and/or modify it under the terms of the GNU Lesser General 
 * Public License as published by the Free Software Foundation; either version 2.1 of the License, or (at your option) 
 * any later version.
 *
 * This library is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied 
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public License for more 
 * details.
 *
 * You should have received a copy of the GNU Lesser General Public License along with this library; if not, write to 
 * the Free Software Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA 
 */

dp.sh.Brushes.MLN = function()
{

    var keywords = 
    'true false';

    this.regexList = [
        { regex: dp.sh.RegexLib.SingleLineCComments,                css: 'comment' },           // one line comments
        { regex: dp.sh.RegexLib.MultiLineCComments,                 css: 'comment' },           // multiline comments
        { regex: dp.sh.RegexLib.DoubleQuotedString,                 css: 'string' },            // strings
        { regex: dp.sh.RegexLib.SingleQuotedString,                 css: 'string' },            // strings
        { regex: new RegExp('^ *#.*', 'gm'),                        css: 'preprocessor' },
        { regex: new RegExp('(\\w+)(\\_\\w+)?(?=\\()', 'gm'),                 css: 'datatypes' },
        { regex: new RegExp('(\\+?\\?\\w+)', 'gm'),                 css: 'variables' },
        { regex: new RegExp(this.GetKeywords(keywords), 'gm'),      css: 'keyword' },
        { regex: new RegExp("(\n|^)[+-]?\\d+(\\.\\d+)?", 'g'), css: 'weight' }
        ];

    this.CssClass = 'dp-mln';
    this.Style =    '.dp-mln .datatypes { color: #000000; font-weight: bold; }' +
                    '.dp-mln .weight { color: #9344b8; }' +
                    '.dp-mln .variables { color: #000000; font-style: italic; }' +
                    '.dp-mln .preprocessor { color: #0f48bd; font-weight: bold; }';
};

dp.sh.Brushes.MLN.prototype = new dp.sh.Highlighter();
dp.sh.Brushes.MLN.Aliases   = ['mln'];
