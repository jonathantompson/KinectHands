if( ! window.MS){
    window.MS = {};
}
MS.Support = MS.Support || {};
MS.Support.AC = function(options){
    if(typeof options === "undefined"){
        return null;
    }
    return this.CreateAC(options);
};
if( ! MS.Support.AC.acarray){
    MS.Support.AC.acarray = [];
}
MS.Support.AC.ACArrayEl = function(el){
    for(var i = 0; i < MS.Support.AC.acarray.length; i ++ ){
        if(MS.Support.AC.acarray[i].key === el){
            return MS.Support.AC.acarray[i].val;
        }
    }
    return null;
};
MS.Support.AC.MakeAutoComplete = function(options){
    if(options.id){
        var newac = new MS.Support.AC(options);
        MS.Support.AC.acarray.push({
            key : options.id,
            val : newac
        });
    }
};
MS.Support.AC.SetCookie = function(key, val){
    var d = new Date();
    d.setFullYear(d.getFullYear() + 1);
    var localdomain = document.domain;
    if(localdomain.indexOf(".com") >- 1){
        document.cookie = key + '=' + val + '; expires=' + d.toGMTString() + '; Domain=' + localdomain + '; path=/';
    }
    else{
        document.cookie = key + '=' + val + '; expires=' + d.toGMTString() + '; path=/';
    }
};
MS.Support.AC.FetchCookie = function fetchEMTCookie(key){
    var cookiename;
    var cookieval;
    var keyfound = false;
    var cookiearray = document.cookie.split(';')
    for(var i = 0; i < cookiearray.length; i ++ ){
        cookiename = cookiearray[i].substring(0, cookiearray[i].indexOf('='));
        if(cookiename.charAt(0) == ' ')
            cookiename = cookiename.substring(1, cookiename.length);
        cookieval = cookiearray[i].substring(cookiearray[i].indexOf('=') + 1, cookiearray[i].length);
        if(key == cookiename){
            keyfound = true;
            break;
        }
    }
    if(keyfound){
        return cookieval;
    }
    else{
        return 'blank';
    }
};
MS.Support.AC.AddEvent = function(el, op, fnc){
    if(el.attachEvent){
        el.attachEvent("on" + op, fnc);
    }
    else{
        el.addEventListener(op, fnc, true);
    }
};
MS.Support.AC.KillEvent = function(el, op, fnc){
    if(el.detachEvent){
        el.detachEvent("on" + op, fnc);
    }
    else{
        el.removeEventListener(op, fnc, true);
    }
};
MS.Support.AC.ACOutputSuggestions = function(query, output, element){
    MS.Support.AC.ACArrayEl(element).OutputSuggestions(query, output);
};
MS.Support.AC.ACErrorSuggestions = function(query, output, element){
    MS.Support.AC.ACArrayEl(element).ErrorSuggestions(query, output);
};
MS.Support.AC.ACSetLcid = function(element, newLcid){
    MS.Support.AC.ACArrayEl(element).options.lcid = newLcid;
};
MS.Support.AC.ACChangeCharStart = function(element, chars){
    MS.Support.AC.ACArrayEl(element).options.startChar = chars;
};
MS.Support.AC.prototype =
{

    CreateAC: function (opts) {
        this.options =
        {
            id: '',
            lcid: '',
            acURL: '',
            statsObj: '',
            startChar: 4,
            bubbleside: 'left',
            dropdirection: 'down',
            dropupgap: 0,
            autoFocus: false,

            lndir: 'ltr',
            acOldValue: '',
            acNewValue: '',
            doNextCall: true,
            acNoVal: '',
            actimer: null,
            acgrowtimer: null,
            dobubble: true,
            acbubbleintimer: null,
            acbubbleouttimer: null,
            bubblecharlimit: 255,
            btopac: 0,
            acQueryBox: null,
            acListDiv: null,
            acListWrap: null,
            acNode: null,
            acIframeHack: null,
            ACEllipsis: null,
            BubbleImgPath: null,
            ACBubbleTextNode: null,
            ACBubbleIframeHack: null,
            acMinWidth: 150,
            acWidth: 'parent',
            ACGrowAmount: 18,
            acReservedKeys: [13, 38, 40],
            nonCharKeys: [9, 16, 17, 18, 19, 20, 27, 33, 34, 35, 36, 37, 39, 46, 112, 113, 114, 115, 116, 117, 118, 119, 120, 121, 122, 123],
            achead: document.getElementsByTagName('head').item(0),
            imgpath: '/library/images/support/en-us/',
            offshiftList: 0,
            borderColor: '#666666',
            charbolding: 2,
            clrwin: (navigator.userAgent.indexOf('AppleWebKit') == -1) ? 'Window' : '#FFF',
            clrwintxt: (navigator.userAgent.indexOf('AppleWebKit') == -1) ? '#333333' : '#000',
            clrhigh: (navigator.userAgent.indexOf('AppleWebKit') == -1) ? 'Highlight' : '#CCC',
            clrhightxt: (navigator.userAgent.indexOf('AppleWebKit') == -1) ? 'HighlightText' : '#000',
            handleResize: null
        };
        this.ACOptionsFill(this.options, opts || {});
        var q,
        o = this.options;
        if (!o) {
            return false;
        }
        if (typeof (o.id) === 'string') {
            q = document.getElementById(o.id);
        }
        else if (typeof (o.id) === 'object' && o.id.nodeName == 'INPUT') {
            q = o.id;
        }
        else {
            return false;
        }
        this.options.lndir = this.ACComputedStyle(q, "direction");
        if (o.bubbleside && typeof (o.bubbleside) === 'string' && o.bubbleside == 'left') {
            o.bubbleside = (o.lndir == 'rtl') ? 'right' : 'left';
        }
        else if (o.bubbleside && typeof (o.bubbleside) === 'string' && o.bubbleside == 'right') {
            o.bubbleside = (o.lndir == 'rtl') ? 'left' : 'right';
        }
        else {
            o.bubbleside = 'right';
        }
        if (o.lcid === '' || typeof (o.lcid) === 'undefined') {
            if (!o.lcid || typeof (o.lcid) !== 'string') {
                o.lcid = '1033';
            }
        }
        if (o.acURL === '' || typeof (o.acURL) === 'undefined') {
            o.acURL = 'http://autocomplete.support.microsoft.com';
        }
        else {
            o.acURL = unescape(o.acURL);
        }
        if (o.startChar && (typeof (o.startChar) === 'string' || typeof (o.startChar) === 'number')) {
            try {
                o.startChar = parseInt(o.startChar, 10);
                if (o.startChar < 2) {
                    o.startChar = 2;
                }
                if (o.startChar > 8) {
                    o.startChar = 8;
                }
            }
            catch (ex) {
                o.startChar = 4;
            }
        }
        if (q) {
            var idTxtFld = document.createElement('input');
            idTxtFld.setAttribute('type', 'hidden');
            idTxtFld.setAttribute('id', 'idTxtFld');
            q.parentNode.insertBefore(idTxtFld, q);
            if (q.offsetWidth < o.acMinWidth) {
                o.acMinWidth = q.offsetWidth;
            }
            q.setAttribute('autocomplete', 'off');
            var a = document.createElement('DIV');
            var as = a.style;
            a.id = 'acListDiv' + q.id;
            as.margin = as.padding = '0px';
            as.marginTop = '-2px';
            as.visibility = 'hidden';
            as.zIndex = '99';
            as.position = 'absolute';
            as.lineHeight = 18 + 'px';
            if (this.options.dropdirection === 'up') {
                as.bottom = (q.offsetHeight + parseInt((q.style.borderWidth * 2), 10) + parseInt((q.style.paddingTop * 2), 10) + this.options.dropupgap) + 'px';
            }
            var wid = parseInt(o.acMinWidth, 10);
            as.width = (wid > 12 ? wid - 2 - 11 : 0) + 'px';
            as.border = '1px solid ' + o.borderColor;
            as.overflow = 'hidden';
            as.padding = '10px 1px 10px 10px';
            as.background = '#fff';
            as.fontSize = '0.7em';
            as.fontFamily = this.ACComputedStyle(q, "fontFamily");
            if (this.options.lndir == 'rtl') {
                as.paddingRight = 10 + 'px';
                as.paddingLeft = 1 + 'px';
            }
            var acw = document.createElement('div');
            acw.id = 'acListWrapper' + q.id;
            acw.style.position = 'relative';
            acw.style.width = parseInt(wid, 10) + 'px';
            acw.style.height = '1px';
            acw.appendChild(a);
            q.parentNode.appendChild(acw);
            o.acNode = document.createElement("div");
            as = o.acNode.style;
            o.acNode.className = "acSugDiv";
            as.whiteSpace = "nowrap";
            as.width = '100%';
            as.overflow = "hidden";
            as.backgroundColor = o.clrwin;
            as.color = o.clrwintxt;
            as.cursor = 'pointer';
            o.ACEllipsis = document.createElement('span');
            o.ACEllipsis.style.background = o.clrwin;
            o.ACEllipsis.style.position = 'absolute';
            o.ACEllipsis.innerHTML = '...';
            o.ACEllipsis.className = 'ellipsis';
            o.ACEllipsis.style.lineHeight = 16 + 'px';
            o.ACEllipsis.style.height = 18 + 'px';
            if (o.lndir == 'ltr') {
                o.ACEllipsis.style.right = '-1px';
            }
            else {
                o.ACEllipsis.style.left = '-1px';
            }
            var btt = document.createElement('div');
            var btb = document.createElement('div');
            var bttxt = document.createElement('div');
            var btarrow = document.createElement('div');
            var btw = document.createElement('div');
            btt.id = 'gss_ac_bttop';
            btb.id = 'gss_ac_btbottom';
            bttxt.id = 'gss_ac_bttxt';
            btarrow.id = 'gss_ac_btarrow';
            btw.id = 'gss_ac_btdiv';
            if (!o.BubbleImgPath) {
                o.BubbleImgPath = o.acURL;
            }
            btt.style.background = 'url(' + o.BubbleImgPath + '/images/bubble.png) no-repeat';
            btb.style.background = 'url(' + o.BubbleImgPath + '/images/bubble.png) 0px -11px no-repeat';
            bttxt.style.background = '#FCFCDD';
            bttxt.style.border = '1px solid #DEBB4F';
            bttxt.style.borderWidth = '0px 1px';
            btb.style.lineHeight = btt.style.lineHeight = btt.style.height = btb.style.height = '10px';
            btt.style.fontSize = btb.style.fontSize = '0px';
            btt.style.width = btb.style.width = '200px';
            bttxt.style.width = '178px';
            bttxt.style.position = btt.style.position = btb.style.position = 'relative';
            bttxt.style.color = 'black';
            bttxt.style.fontSize = '.7em';
            bttxt.style.fontFamily = a.style.fontFamily;
            bttxt.style.overflowX = 'hidden';
            bttxt.style.whiteSpace = 'normal';
            bttxt.style.padding = '0px 10px';
            btw.style.position = btarrow.style.position = 'absolute';
            btarrow.style.top = '0px';
            btarrow.style.height = '23px';
            btarrow.style.width = '34px';
            btw.style.width = '226px';
            btw.style.top = '90px';
            btw.style.zIndex = 999;
            if (o.bubbleside == 'left') {
                btt.style.cssFloat = btb.style.cssFloat = bttxt.style.cssFloat = btt.style.styleFloat = btb.style.styleFloat = bttxt.style.styleFloat = 'left';
                btw.style.left = '-220px';
                btarrow.style.right = '0px';
                btarrow.style.background = 'url(' + o.BubbleImgPath + '/images/bubble.png) 0px -22px no-repeat';
            }
            else {
                btw.style.left = a.offsetWidth + 'px';
                btarrow.style.left = '1px';
                btarrow.style.background = 'url(' + o.BubbleImgPath + '/images/bubble.png) -35px -22px no-repeat';
                btt.style.cssFloat = btb.style.cssFloat = bttxt.style.cssFloat = btt.style.styleFloat = btb.style.styleFloat = bttxt.style.styleFloat = 'right';
            }
            btw.appendChild(btt);
            btw.appendChild(bttxt);
            btw.appendChild(btb);
            btw.appendChild(btarrow);
            o.ACBubbleTextNode = btw;
            if (navigator.userAgent.indexOf('MSIE') > 0) {
                if (parseInt((navigator.userAgent.charAt(navigator.userAgent.indexOf('MSIE') + 5)), 10) < 7) {
                    this.ACMakeIframeHack(acw, a);
                    var btifram = document.createElement('iframe');
                    btifram.id = 'gss_ac_btiframe';
                    btifram.style.width = '226px';
                    btifram.style.position = 'absolute';
                    btifram.style['filter'] = 'alpha(opacity=0)';
                    if (o.bubbleside == 'left') {
                        btifram.style.left = '-220px';
                    }
                    else {
                        btifram.style.right = '-220px';
                    }
                    o.ACBubbleIframeHack = btifram;
                }
            }
            var t = this;
            this.ACAddEvt(q, "keyup", function (event) {
                t.ACInitSuggestions(event);
            });
            this.ACAddEvt(q, "keydown", function (event) {
                t.ACHandleKeysDown(event);
            });
            this.ACAddEvt(document, "mouseup", function (event) {
                t.ACClickHandler(event);
            });
            o.acListDiv = a;
            o.acQueryBox = q;
            o.acListWrap = acw;
            if (o.autoFocus && q.createTextRange && !document.location.hash) {
                this.ACFocus();
            }
        }
        return this;
    },
    ACFocus: function () {
        var o = this.options;
        var q = o.acQueryBox;
        var len = q.value.length;
        if (q.createTextRange) {
            var r = q.createTextRange();
            try {
                if (o.lndir === 'rtl') {
                    r.move("character", -len);
                } else {
                    r.move("character", len);
                }
                r.select();
            }
            catch (e) { }
        } else {
            q.focus();
            if (q.setSelectionRange) {
                if (o.lndir === 'rtl') {
                    q.setSelectionRange(0, 0);
                } else {
                    q.setSelectionRange(len, len);
                }
            }
        }
    },
    ACOptionsFill: function (newOptions, defaultOptions) {
        for (option in defaultOptions) {
            newOptions[option] = defaultOptions[option];
        }
        return newOptions;
    },
    ACDoShowTarget: function () {
        clearTimeout(this.options.acgrowtimer);
        if (this.options.acListDiv.style.visibility != 'visible') {
            this.options.acListDiv.style.height = '18px';
            this.options.acListDiv.style.visibility = 'visible';
            this.ACIframeVisible(this.options.acListDiv.style.visibility);
        }
        var t = this;
        this.options.acgrowtimer = setInterval(function (event) {
            t.ACGrowWindow(event);
        }, 10);
    },
    ACDoHideTarget: function () {
        clearTimeout(this.options.acgrowtimer);
        if (this.options.acListDiv.style.visibility != 'hidden') {
            var t = this;
            this.PopBubble();
            this.options.acgrowtimer = setInterval(function (event) {
                t.ACShrinkWindow(event);
            }, 10);
        }
    },
    ACDropWidth: function () {
        var wid;
        if (typeof (this.options.acWidth) === 'number') {
            wid = this.options.acWidth;
        }
        else if (typeof (this.options.acWidth) === 'string' && this.options.acWidth == 'self') {
            wid = (this.options.acQueryBox.offsetWidth && this.options.acQueryBox.offsetWidth > this.options.acMinWidth) ? (this.options.acQueryBox.offsetWidth) : this.options.acMinWidth;
        }
        else {
            wid = (this.options.acQueryBox.parentNode.offsetWidth && this.options.acQueryBox.parentNode.offsetWidth > this.options.acMinWidth) ? (this.options.acQueryBox.parentNode.offsetWidth) : this.options.acMinWidth;
        }
        wid = parseInt(wid, 10) - 2 - 11;
        return (wid >= 0 ? wid : 0) + 'px';
    },
    ACGrowWindow: function () {
        var ht = 0;

        for (var i = 0; i < parseInt(this.options.acListDiv.childNodes.length, 10); i++) {
            ht += parseInt(this.options.acListDiv.childNodes[i].offsetHeight, 10);
        }
        if (ht) {
            var flip = (ht < this.options.acListDiv.offsetHeight) ? -1 : 1;
            var newht = (parseInt(this.options.acListDiv.style.height, 10) + (this.options.ACGrowAmount * flip));
            this.options.acListDiv.style.height = newht + 'px';
            this.ACIframeSize(newht);
            if (newht >= (ht - this.options.ACGrowAmount)) {
                this.options.acListDiv.style.height = ht + 'px';
                this.ACIframeSize(ht);
                clearTimeout(this.options.acgrowtimer);
            }
        }
        else {
            this.options.acListDiv.style.height = 'auto';
            this.ACIframeSize(this.options.acListDiv.style.height);
            clearTimeout(this.options.acgrowtimer);
        }
        if (this.options.handleResize)
            this.options.handleResize(this.options.acListDiv);
    },
    ACShrinkWindow: function (ht) {
        if (this.options.acListDiv.style.height != 'auto' && this.options.acListDiv) {
            var hght = parseInt(this.options.acListDiv.style.height, 10) - 12;
            hght = (hght > 0) ? hght : 0;
            this.options.acListDiv.style.height = hght + 'px';
            this.ACIframeSize(this.options.acListDiv.style.height);
            if (parseInt(this.options.acListDiv.style.height, 10) < 13) {
                this.options.acListDiv.style.visibility = 'hidden';
                this.ACIframeVisible(this.options.acListDiv.style.visibility);
                clearTimeout(this.options.acgrowtimer);
            }
        }
        else {
            this.options.acListDiv.style.visibility = 'hidden';
            this.options.acListDiv.style.height = '1px';
            this.ACIframeVisible(this.options.acListDiv.style.visibility);
            this.ACIframeSize(this.options.acListDiv.style.height);
            clearTimeout(this.options.acgrowtimer);
        }
        if (this.options.handleResize)
            this.options.handleResize(this.options.acListDiv);
    },

    ACGetSuggestions: function () {
        var temp = document.getElementById('acScript');
        if (temp) {
            this.options.achead.removeChild(temp);
        }
        var script = document.createElement('SCRIPT');
        script.id = 'acScript';
        script.setAttribute("type", "text/javascript");
        script.src = this.options.acURL + '/ACSearchSuggest.aspx?eleid=' + this.options.acQueryBox.id + '&lcid=' + this.options.lcid + '&query=' + encodeURIComponent(this.options.acOldValue);
        this.options.achead.appendChild(script);
    },
    ACClickHandler: function (e, forceAction) {
        var el = this.ACSrcEl(e);
        var ic = this.ACIsChild(el, this.options.acListDiv);
        if (el === this.options.acQueryBox && !forceAction) {
            return false;
        }
        if (!ic && (!this.options.handleResize || !this.ACIsChild(el, this.options.acListDiv.parentNode))) {
            this.ACHideTarget();
            return false;
        }
        var returnValue = true;
        if (this.options.acListDiv) {
            var q = this.options.acQueryBox;
            var n = this.options.acListDiv.childNodes[q.selectedIndex];
            if (ic && n) {
                q.previousSibling.disabled = false;
                q.previousSibling.setAttribute('name', 'qid');
                q.previousSibling.value = n.getAttribute('suggestionId');
                q.value = n.getAttribute('textVal');
                q.focus();
                this.ACSelectText(e);
                this.options.acOldValue = q.value;
                if (this.options.statsObj) {
                    this.options.statsObj.eventCollectionId = SetLogCollectionBit(this.options.statsObj.eventCollectionId, 41);
                }
                returnValue = false;
            }
            if (this.options.acListDiv && ic) {
                this.ACHideTarget();
            }
        }
        return returnValue;
    },
    ACInitSuggestions: function (e) {
        for (var i = 0; i < this.options.acReservedKeys.length; i++) {
            if (this.ACGetKeyCode(e) == this.options.acReservedKeys[i]) {
                return;
            }
        }
        var q = this.options.acQueryBox;
        if (this.ACGetKeyCode(q) == 9) {
            this.ACClickHandler(q);
        }
        q.selectedIndex = -1;
        this.ACHighlight();
        this.options.acNewValue = (q.value) ? q.value.ACRemoveSpaces() : "";
        if (this.options.acNoVal !== "") {
            if (this.options.acNoVal.toLowerCase() == this.options.acNewValue.substring(0, this.options.acNoVal.length).toLowerCase()) {
                return;
            }
            else {
                this.options.acNoVal = "";
            }
        }
        for (var i = 0; i < this.options.nonCharKeys.length; i++) {
            if (this.ACGetKeyCode(e) == this.options.nonCharKeys[i]) {
                q.previousSibling.disabled = false;
                return;
            }
        }
        q.previousSibling.disabled = true;
        if (this.options.acOldValue.toLowerCase() != this.options.acNewValue.toLowerCase() && this.options.acNewValue.length >= this.options.startChar) {
            if (this.options.actimer) {
                clearTimeout(this.options.actimer);
            }
            var t = this;
            if (!this.options.doNextCall) {
                var re = new RegExp('^' + this.options.acOldValue.RemoveRegexChrs(), 'i');
                if (this.options.acNewValue.match(re)) {
                    this.ACHideTarget();
                }
                else {
                    this.options.doNextCall = true;
                }
            }
            if (this.options.doNextCall) {
                this.options.acOldValue = this.options.acNewValue;
                this.options.actimer = setTimeout(function (event) {
                    t.ACGetSuggestions(event);
                }, 100);
            }
        }
        else if (this.options.acNewValue.length < this.options.startChar) {
            this.options.doNextCall = true;
            this.ACHideTarget();
            this.options.acOldValue = this.options.acNewValue;
        }
    },
    ErrorSuggestions: function (query, error) {
        return;
    },
    OutputSuggestions: function (query, output) {
        try {
            this.PopBubble();
            if (output === "") {
                this.options.acNoVal = query;
                this.ACHideTarget();
                return;
            }
            else {
                this.options.acNoVal = "";
            }
            if (this.options.statsObj) {
                this.options.statsObj.eventCollectionId = SetLogCollectionBit(this.options.statsObj.eventCollectionId, 40);
                this.options.statsObj.flexId = 11;
                this.options.statsObj.flexValue1 = query.substr(0, 256);
            }
            var sg = [];
            for (var i = 0; i < output.length; i += 2) {
                sg.push(this.ACSuggestion(output[i], output[i + 1]));
            }
            sg = this.ACSort(sg);
            if (sg.length === 0) {
                this.options.doNextCall = false;
                this.ACHideTarget();
                return;
            }
            var t = this;
            this.options.acListDiv.innerHTML = "";
            query = query.RemoveRegexChrs();
            for (var i = 0; i < sg.length; i++) {
                var d0 = this.options.acNode.cloneNode(true);
                var dtxt = document.createElement('span');
                d0.setAttribute('suggestionId', sg[i].ID);
                d0.setAttribute('textVal', sg[i].Suggestion);
                d0.setAttribute('queryVal', query);
                dtxt.className = 'gss_ac_dtxt';
                dtxt.id = dtxt.className + i;
                dtxt.innerHTML = this.BoldChars(sg[i].Suggestion, query);
                d0.appendChild(dtxt);
                this.ACAddEvt(d0, "mouseover", function (event) {
                    t.acMouseHighlight(event);
                });
                this.ACAddEvt(dtxt, "mouseover", function (event) {
                    t.acMouseHighlight(event);
                });
                this.ACAddEvt(d0, "keyup", function (event) {
                    t.BlowBubble(event);
                });
                this.ACAddEvt(d0, "mouseover", function (event) {
                    t.BlowBubble(event);
                });
                this.options.acListDiv.appendChild(d0);


                if (dtxt.offsetWidth > d0.offsetWidth) {
                    d0.setAttribute('balloon', true);
                    d0.appendChild(this.options.ACEllipsis.cloneNode(true));
                }
            }
            this.ACDoShowTarget();
            var acwid = this.ACDropWidth();
            this.options.acListDiv.parentNode.style.width = (parseInt(acwid) - 1) + 'px';
            this.options.acListDiv.style.width = acwid;
        }
        catch (ex) {
            this.ACHideTarget();
        }
    },
    BoldChars: function (sstr, bstr) {
        var ostr = sstr;
        try {
            ostr = ostr.replace(/</gi, '&lt;').replace(/>/gi, '&gt;');
            bstr = bstr.replace(/</gi, '&lt;').replace(/>/gi, '&gt;');
            switch (this.options.charbolding) {
                case 1:

                    var re = new RegExp(bstr, 'i');
                    ostr = ostr.replace(re, '<b>$&</b>');
                    break;
                case 2:

                    var re = new RegExp(bstr, 'gi');
                    ostr = ostr.replace(re, '<b>$&</b>');
                    break;
                case 3:

                    var word = bstr.split(/\s+/g);
                    var i = 0;
                    for (; i < word.length - 1; i++) {
                        var re = new RegExp('(^|\\s)(?!<|</|<b>)(' + word[i] + ')(?!>|<\/b>)(\\b|\\s)', 'gi');
                        ostr = ostr.replace(re, '$1<b>$2</b>$3');
                    }
                    var re = new RegExp('(^|\\s)(?!<|</|<b>)(' + word[i] + ')(?!>|<\/b>)', 'gi');
                    ostr = ostr.replace(re, '$1<b>$2</b>');
                    break;
                case 4:

                    var word = bstr.split(/\s+/g);
                    for (var i = 0; i < word.length; i++) {
                        var re = new RegExp('(?!<|</|<b>)(' + word[i] + ')(?!>|<\/b>)', 'gi');
                        ostr = ostr.replace(re, '<b>$&</b>');
                    }
                    break;
                default:
            }
        }
        catch (ex) {
            return sstr;
        }
        return ostr;
    },
    BlowBubble: function (event) {
        if (!this.options.dobubble) {
            return;
        }
        var el = this.ACSrcEl(event);
        while (el.tagName == 'SPAN' || el.tagName == 'B') {
            el = el.parentNode;
        }

        if (!el.getAttribute('balloon')) {
            this.PopBubble();
            return;
        }
        var bt = document.getElementById('gss_ac_btdiv');
        var bti = document.getElementById('gss_ac_btiframe');
        if (!bt) {
            this.options.acListDiv.parentNode.appendChild(this.options.ACBubbleTextNode);
            bt = document.getElementById('gss_ac_btdiv');
            if (this.options.ACBubbleIframeHack) {
                this.options.acListDiv.parentNode.appendChild(this.options.ACBubbleIframeHack);
                bti = document.getElementById('gss_ac_btiframe');
            }
            var t = this;
            bt.style.opacity = this.options.btopac;
            bt.style['filter'] = 'progid:DXImageTransform.Microsoft.Alpha(style=0, opacity=' + this.options.btopac + ')';
            clearInterval(this.acbubbleintimer);
            this.options.acbubbleintimer = setInterval(function (event) {
                t.FadeInBubble();
            }, 5);
        }
        document.getElementById('gss_ac_bttxt').innerHTML = this.CharSubset(el.getAttribute('textVal'), el.getAttribute('queryVal'));
        bt.style.top = (el.offsetTop + (el.offsetHeight / 2)) + 'px';
        if (this.options.bubbleside == 'right') {
            bt.style.left = this.options.acListDiv.style.width;
        }
        if (bti) {
            bti.style.top = bt.style.top;
            bti.style.height = bt.offsetHeight + 'px';
        }
    },
    PopBubble: function () {
        var bt = document.getElementById('gss_ac_btdiv');
        if (bt && this.options.acbubbleouttimer == null) {
            clearInterval(this.options.acbubbleouttimer);
            var t = this;
            this.options.btopac = 100;
            bt.style.opacity = this.options.btopac / 100;
            bt.style['filter'] = 'progid:DXImageTransform.Microsoft.Alpha(style=0, opacity=' + this.options.btopac + ')';
            this.options.acbubbleouttimer = setInterval(function (event) {
                t.FadeOutBubble();
            }, 5);
        }
    },
    FadeInBubble: function () {
        var bt = document.getElementById('gss_ac_btdiv');
        if (bt) {
            this.options.btopac = this.options.btopac + 10;
            bt.style.opacity = this.options.btopac / 100;
            bt.style['filter'] = 'progid:DXImageTransform.Microsoft.Alpha(style=0, opacity=' + this.options.btopac + ')';
            if (this.options.btopac >= 100) {
                clearInterval(this.options.acbubbleintimer);
                this.options.btopac = 0;
            }
        }
    },
    FadeOutBubble: function () {
        var bt = document.getElementById('gss_ac_btdiv');
        var bti = document.getElementById('gss_ac_btiframe');
        if (bt) {
            this.options.btopac = this.options.btopac - 10;
            bt.style.opacity = this.options.btopac / 100;
            bt.style['filter'] = 'progid:DXImageTransform.Microsoft.Alpha(opacity=' + this.options.btopac + ')';
            if (this.options.btopac <= 0) {
                clearInterval(this.options.acbubbleouttimer);
                this.options.acbubbleouttimer = null;
                this.options.btopac = 0;
                if (bti) {
                    bti.parentNode.removeChild(bti);
                }
                bt.parentNode.removeChild(bt);
            }
        }
    },
    CharSubset: function (otxt, btxt) {
        var stxt = otxt.substring(0, this.options.bubblecharlimit);
        if (otxt.length > this.options.bubblecharlimit) {
            stxt += '...';
        }
        return stxt;
    },
    ACSort: function (arr) {
        var srt = function (a, b) {
            return a.Suggestion.length - b.Suggestion.length;
        };
        return arr.sort(srt);
    },
    ACAddEvt: function (el, op, fnc) {
        MS.Support.AC.AddEvent(el, op, fnc);
    },
    ACKillEvt: function (el, op, fnc) {
        MS.Support.AC.KillEvent(el, op, fnc);
    },
    ACHandleKeysDown: function (e) {
        var k = this.ACGetKeyCode(e);
        var q = this.ACSrcEl(e);
        var acwid = this.ACDropWidth();
        this.options.acListDiv.parentNode.style.width = (parseInt(acwid) - 1) + 'px';
        this.options.acListDiv.style.width = acwid;
        if (this.options.acListDiv && this.options.acListDiv.style && this.options.acListDiv.style.visibility != "hidden") {
            var max = this.options.acListDiv.childNodes.length;
            var si = q.selectedIndex;
            var n = this.options.acListDiv.childNodes;
            if (k == 40) {
                if (si < max - 1) {
                    q.selectedIndex++;
                }
                else if (si >= max - 1) {
                    q.selectedIndex = -1;
                }
                si = q.selectedIndex;
                this.ACHighlight();
                if (si < 0) {
                    q.value = this.options.acOldValue;
                    this.PopBubble();
                }
                else {
                    var currentText = n[si].getAttribute('textVal');
                    if (currentText != null) {
                        q.previousSibling.disabled = false;
                        q.previousSibling.setAttribute('name', 'qid');
                        q.previousSibling.value = n[si].getAttribute('suggestionId');
                        q.value = currentText;
                    }
                    this.BlowBubble(n[si]);
                }
                this.ACSelectText(e);
                e.returnValue = false;
                return false;
            }
            else if (k == 38) {
                if (si >= 0) {
                    q.selectedIndex--;
                }
                else if (si == -1) {
                    q.selectedIndex = max - 1;
                }
                si = q.selectedIndex;
                this.ACHighlight();
                if (si < 0) {
                    q.value = this.options.acOldValue;
                    this.PopBubble();
                }
                else {
                    var currentText = n[si].getAttribute('textVal');
                    if (currentText != null)
                        q.value = currentText;
                    this.BlowBubble(n[si]);
                }
                this.ACSelectText(e);
                e.returnValue = false;
                return false;
            }
            else if (k == 13 && si > -1) {
                e.returnValue = this.ACClickHandler(n[si]);
                this.ACStopDefault(e);
                return e.returnValue;
            }
            else if (k == 27) {
                this.ACHideTarget();
                q.value = this.options.acOldValue;
                q.focus();
                this.ACSelectText(e);
                e.returnValue = false;
                return false;
            }
            else if (k == 9) {
                this.ACClickHandler(e, true);
            }
        }
        else {
            if (k == 27) {
                q.value = '';
                q.focus();
                this.ACSelectText(e);
                e.returnValue = false;
                return false;
            }
        }
    },

    ACSelectText: function (event) {
        if (!this.options.acOldValue) {
            return;
        }
        if (this.options.acQueryBox.createTextRange) {
            var textRange = this.options.acQueryBox.createTextRange();
            textRange.moveStart("character", this.options.acOldValue.length);
            textRange.select();
        }
        else if (this.options.acQueryBox.setSelectionRange) {
            this.options.acQueryBox.focus();
            this.options.acQueryBox.setSelectionRange(this.options.acOldValue.length, this.options.acQueryBox.value.length);
        }
        this.ACStopDefault(event);
    },
    acMouseHighlight: function (e) {
        var el = this.ACSrcEl(e);
        while (el.tagName == 'SPAN' || el.tagName == 'B') {
            el = el.parentNode;
        }
        var sibs = this.options.acListDiv.childNodes;
        for (var i = 0; i < sibs.length; i++) {
            if (sibs[i] == el) {
                this.options.acQueryBox.selectedIndex = i;
            }
        }
        this.ACHighlight();
    },
    ACHighlight: function () {
        var sibs = this.options.acListDiv.childNodes;
        for (var i = 0; i < sibs.length; i++) {
            if (sibs[i].className == 'acSugDiv') {
                if (i != this.options.acQueryBox.selectedIndex) {
                    this.HighlightEls(sibs[i], 'background', this.options.clrwin);
                    this.HighlightEls(sibs[i], 'color', this.options.clrwintxt);
                }
                else {
                    this.HighlightEls(sibs[i], 'background', this.options.clrhigh);
                    this.HighlightEls(sibs[i], 'color', this.options.clrhightxt);
                }
            }
        }
    },
    HighlightEls: function (el, style, color) {
        if (el && el.tagName) {
            el.style[style] = color;
            if (el.childNodes) {
                for (var i = 0; i < el.childNodes.length; i++) {
                    this.HighlightEls(el.childNodes[i], style, color);
                }
            }
        }
    },
    ACStopDefault: function (event) {
        if (window.event) {
            window.event.cancelBubble = true;
            window.event.returnValue = false;
        }
        else {
            if (event.preventDefault) {
                event.preventDefault();
            }
        }
    },
    ACHideTarget: function () {
        if (this.options.acListDiv) {
            this.ACDoHideTarget();
            this.options.acQueryBox.selectedIndex = -1;
        }
    },
    ACGetKeyCode: function (e) {
        if (!e) {
            return null;
        }
        if (e.keyCode) {
            return e.keyCode;
        }
        if (e.charCode) {
            return e.charCode;
        }
        return null;
    },
    ACComputedStyle: function (el, style) {
        var returnVal = "";
        if (document.defaultView && document.defaultView.getComputedStyle) {
            returnVal = document.defaultView.getComputedStyle(el, null)[style];
        }
        else if (el.currentStyle) {
            style = style.replace(/-(w)/g, function (strMatch, p1) {
                return p1.toUpperCase();
            });
            returnVal = el.currentStyle[style];
        }
        return returnVal;
    },
    ACIsChild: function (child, parent) {
        if (!child || !parent) {
            return false;
        }
        var pe = child.parentNode;
        if (parent == pe) {
            return true;
        }
        if (!pe) {
            return false;
        }
        return this.ACIsChild(pe, parent);
    },
    ACSrcEl: function (e) {
        if (e) {
            if (e.srcElement) {
                return e.srcElement;
            }
            if (e.target) {
                return e.target;
            }
        }
        return e;
    },


    ACMakeIframeHack: function (parent, a) {
        if (!this.options.acIframeHack) {
            this.options.acIframeHack = document.createElement('IFRAME');
            var i = this.options.acIframeHack.style;
            var as = a.style;
            i.marginTop = as.marginTop;
            i.left = as.left;
            i.top = as.top;
            i.visibility = as.visibility;
            i.width = '100%';
            i.height = '0px';
            i.position = as.position;
            this.options.acIframeHack.style['filter'] = 'progid:DXImageTransform.Microsoft.Alpha(style=0,opacity=0)';
            this.options.acIframeHack.src = "";
            parent.appendChild(this.options.acIframeHack);
            this.ACIframeSize = function (ht) {
                i.height = ht;
            };
            this.ACIframeVisible = function (vis) {
                i.visibility = vis;
            };
        }
    },
    ACSuggestion: function (id, suggestion) {
        var obj =
    {
        ID: id, Suggestion: suggestion
    };
        return obj;
    },
    ACIframeSize: function () { },
    ACIframeVisible: function () { }
};
String.prototype.RemoveRegexChrs = function(){
    var regExChrs = new RegExp('(\\\\|\\.|\\+|\\$|\\?|\\*|\\(|\\)|\\[|\\]|\\^|\\!)', 'g');
    return this.replace(regExChrs, '\\$1');
};
var dbcsSpaceRE = new RegExp(String.fromCharCode(12288), "g");
String.prototype.ACRemoveSpaces = function(){
    var temp = this.replace(dbcsSpaceRE, ' ');
    temp = temp.replace(/^\s+|\s+$/g, '');
    return temp.replace(/\s\s+/g, ' ');
};