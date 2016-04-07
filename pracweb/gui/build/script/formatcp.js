var tabbed = ["contains", "to", "amount", "number", "destination"];
var align = ["from", "liquid"];
var alignType = ["a quantity \\(type", "an object \\(type"];

function formatCP(cramPlan) {
    if (((cramPlan.match(/\(/g) || []).length) < 5) return (cramPlan + "<br><br>");
    var idx;
    var textPre;
    var textPost;
    for (var i = 0; i < tabbed.length; i++) {
        var j = 0;
        do {
            idx = cramPlan.indexOf("(" + tabbed[i],j);
            if (idx == -1) break;
            textPre = cramPlan.substring(0,idx);
            textPost = cramPlan.substring(idx);
            cramPlan = textPre + "<br>" + textPost;
            if (idx < cramPlan.length) j = idx+5;
        } while (idx != cramPlan.lastIndexOf("(" + tabbed[i],j));
    }
    var textSplit = cramPlan.split("<br>");
    var alignArray = [];
    var alignArray2 = [];
    var tempAlignArray = [];
    for (var i = 0; i < textSplit.length; i++) {
        tempAlignArray = [];
        for (var j = 0; j < alignArray.length; j++) {
            if (alignArray2[j] == i-1) {
                if (!j || alignArray2[j-1] != alignArray2[j]) {  
                    textSplit[i-1] = "<div id=\"cf\">" + textSplit[i-1].slice(0,alignArray[j]) + "&nbsp;<br>&nbsp;</div>" + textSplit[i-1].slice(alignArray[j],textSplit[i-1].length);
                    for (var k = 0; k < alignArray2.length; k++) {
                        if (alignArray2[k] == j) {
                            alignArray[k]+=35;
                        } else {
                            break;
                        }
                    }
                } else {
                    textSplit[i-1] = textSplit[i-1].slice(0,alignArray[j-1]) + "<div id=\"cf\">" + textSplit[i-1].slice(alignArray[j-1],alignArray[j]) + "&nbsp;<br>&nbsp;</div>" + textSplit[i-1].slice
(alignArray[j],textSplit[i-1].length);
                    for (var k = j; k < alignArray2.length; k++) {
                        if (alignArray2[k] == j) {
                            alignArray[k]+=35;
                        } else {
                            break;
                        }
                    }
                    
                }
            } else {
                textSplit[alignArray2[j]] = textSplit[alignArray2[j]].slice(0,alignArray[j]-6) + "<br>&nbsp;" + textSplit[alignArray2[j]].slice(alignArray[j]-6,textSplit[alignArray2[j]].length);
                for (var k = j; k < alignArray2.length; k++) {
                    if (alignArray2[k] == j) {
                            alignArray[k]+=10;
                    } else {
                            break;
                    }
                }
            }
        }
        if (alignArray.length>1) {
            alignArray.pop();
            alignArray2.pop();
        }

        for (var j = 0; j < align.length; j++) {
            var tempAlign1 = textSplit[i].search(new RegExp("\\(" + align[j]));
            if (tempAlign1 > 0) {
                alignArray2.push(i);
                tempAlignArray.push(tempAlign1);
                break;
            }
        }
        for (var j = 0; j < alignType.length; j++) {
            var tempAlign2 = textSplit[i].search(new RegExp(alignType[j]));
            if (tempAlign2 > 0) {
                tempAlign2 += alignType[j].length-6;
                alignArray2.push(i);
                tempAlignArray.push(tempAlign2);
                break;
            }
        }
        alignArray = alignArray.concat(tempAlignArray.sort());
    }
    return textSplit.join("<br>") + "<br><br>";
}

