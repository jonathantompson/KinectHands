/*
Variable to store if link is clicked or not
*/
var clicked = false;

// this value will be assigned from the server-side code
var extraOp = '';

var supportedSites = [];

var currentSiteIndex = 0;

var avlCatalogs = [];
var currentCatSelIndx = -1;
var optContainer = null;

var isIE6 = false;
var prevElement = null;

var searchSubmit = 0;

var Microsoft = window.Microsoft || {};
Microsoft.Support = Microsoft.Support || {};
Microsoft.Support.GSS = Microsoft.Support.GSS || {};
Microsoft.Support.GSS.GpsSearch = function (selector, args) {
    var defaultOptions = {
        gpsSearchCountPerPage: 5,
        language: "en-us"
    }
    var options = $.extend(true, {}, defaultOptions, args);
    this.callService = function (pageNumber) {
        var pageRange = "1-" + options.gpsSearchCountPerPage;
        var page = parseInt(pageNumber);
        if (page > 0) {
            var low = (page - 1) * 5 + 1;
            var high = page * options.gpsSearchCountPerPage;
            pageRange = low + "-" + high;
        }
        $(".loading-panel", selector).css("display", "block");
        $(".update-panel", selector).css("display", "none");
        $(".error-panel", selector).css("display", "none");
        var productNode = window._wayfinder.getCurrentProduct();
        $.ajax({
            url: "/GuidedProblemSolving/GuidedProblemSolvingService.svc/GetGPSPageContent",
            cache: false,
            type: "get",
            dataType: "json",
            data: {
                query: $.param({ query: $("#gsfx_bsrch_query").val() }),
                lang: options.language,
                modalityType: window._wayfinder.getCurrentModality().type,
                range: pageRange,
                path: productNode ? productNode.getPath() : ""
            },
            success: function (data) {
                if (typeof data === "undefined" || data === null) {
                    $(".error-panel", selector).css("display", "block");
                    $(".loading-panel", selector).css("display", "none");
                    return;
                }
                $(selector).parent().html(data.d);
            },
            error: function () {
                $(".error-panel", selector).css("display", "block");
                $(".loading-panel", selector).css("display", "none");
                return;
            }
        });
    }
};

function submitSimpleSearch(url, query) {
    var getCurrentSite = function () {
        if (supportedSites && supportedSites.length > 0) {
            return supportedSites[currentSiteIndex];
        }
    }
    var currentSite = getCurrentSite();
    if (currentSite == null) {
        return;
    }
    if (!url || url.length == 0) {
        url = currentSite.Url;
    }

    var clickAction = currentSite.ClickAction;
    switch (clickAction) {
        case ClickAction.FormSubmit:
            srch_setcookieval("lquery", UnicodeFixup(escape(jQuery.trim(query))));
            StatsDotNet.eventCollectionId = SetLogCollectionBit(StatsDotNet.eventCollectionId, 6);
            searchSubmit = 1;
            $("#frmsrch").get(0).submit();
            break;
        case ClickAction.AjaxCall:
            srch_setcookieval("lquery", UnicodeFixup(escape(jQuery.trim(query))));
            window._gps.changeQuery({query:query});
            return false;
        case ClickAction.Redirect:
        default:
            StatsDotNet.OptionCollectionId = SetLogCollectionBit(StatsDotNet.OptionCollectionId, 33);
            StatsDotNet.targetUrl = url + query;

            if (window.encodeURIComponent) {
                query = encodeURIComponent(query);
            }
            else {
                query = OutputEncoder_EncodeUrl(query);
            }
            document.location.href = url + query;
            break;
    }
    return;
}

/*
Called when form is submitted onsearch page
*/
function SubmitSearch(frm) {
    elem = $('#frmaSrch').get(0).query;
    srch_setcookieval("lquery", UnicodeFixup(escape(jQuery.trim(elem.value))));
    if ($('[name=catalog]').get(0) != null) {
        for (s = 0; s < $('[name=catalog]').length; s++) {
            if ($('[name=catalog]').get(s).type != "select-one") {
                if ($('[name=catalog]').get(s).checked) {
                    var msurl;
                    var qry = frm.query.value;
                    qry = jQuery.trim(qry);
                    if (window.encodeURIComponent) { qry = encodeURIComponent(qry); }
                    else { qry = OutputEncoder_EncodeUrl(qry); }
                    if ($('[name=catalog]').get(s).value == 'msc') {
                        StatsDotNet.OptionCollectionId = SetLogCollectionBit(StatsDotNet.OptionCollectionId, 33);

                        msurl = mscomurl + qry;
                        document.location.href = msurl;
                        return false;
                    }
                    else if ($('[name=catalog]').get(s).value == 'msn') {
                        StatsDotNet.OptionCollectionId = SetLogCollectionBit(StatsDotNet.OptionCollectionId, 33);

                        msurl = msnurl + qry;
                        StatsDotNet.targetUrl = msurl;
                        document.location.href = msurl;
                        return false;
                    }
                    else {
                        srch_setcookieval('adcatalog', escape($('[name=catalog]').get(s).value));
                    }
                }
            }
            else {
                var catalogList = $('[name=catalog]');
                if (catalogList && catalogList.options) {
                    var pvalue = catalogList.options[catalogList.selectedIndex].value;
                    if (pvalue != "") {
                        srch_setcookieval('adcatalog', pvalue);
                    }
                }
            }
        }
    }
    else {
        //save catalog in cookie
        var catalogList = $('#catalog').get(0);
        if (catalogList && catalogList.options) {
            var pvalue = catalogList.options[catalogList.selectedIndex].value;
            if (pvalue != "") {
                srch_setcookieval('adcatalog', pvalue);
            }
        }
    }
    SaveSrchState(true);
    StatsDotNet.eventCollectionId = SetLogCollectionBit(StatsDotNet.eventCollectionId, 7);
    PageSubmit = 1;
}

function SaveSrchState(saveQuery) {
    if (PageSubmit === 1) {
        return;
    }

    var elem;

    // save Query Value
    if (saveQuery === true) {
        elem = $('#frmaSrch').get(0).query;
        srch_setcookieval("lquery", UnicodeFixup(escape(jQuery.trim(elem.value))));
    }
    // save catalog.Value
    elem = $('[name=catalog]');
    // Check if there are multiple catalogs
    if (elem && elem.length && elem.type != "select-one") { SaveRadioState(elem, "catalog"); }

    var optcookie = "";
    var optresource = "";
    // save scope options
    if (document.getElementsByName) {
        elem = document.getElementsByName("ast");
        for (i = 0; i < elem.length; i++) {
            if (elem[i].disabled)
                continue;
            var optrow = $('#' + elem[i].value + 'row').get(0);
            if (optrow && (optrow.style.display == "block" || optrow.style.display == "")) {
                if (elem[i].checked) {
                    optcookie += "ad" + elem[i].value + "=1|";
                    optresource += elem[i].value + ",";
                }
                else {
                    optcookie += "ad" + elem[i].value + "=0|";
                }
            }
            else { elem[i].value = ""; }
        }
    }
    if (optElems) {
        options = optElems.split('|');
        if (options != null) {
            for (i = 0; i < options.length; i++) {
                elem = $('#' + options[i].toString()).get(0);
                if (elem) {
                    var radioCatalog = $('#' + elem.attributes['_parentid'].value).get(0);
                    if (radioCatalog.checked) {
                        if (elem.checked) {
                            optcookie += "ad" + options[i] + "=1|";
                            optresource += options[i] + ",";
                        }
                        else { optcookie += "ad" + options[i] + "=0|"; }
                    }
                }
            }
        }

    }
    srch_setcookieval("adresource", optresource);
    srch_setcookieval("adopt", optcookie);
}

// initialize simple search
function InitSrch() {
    if (null == $('#catalog').get(0)) {
        // if the hidden catalog field does not exist - add it to the form
        // this will happen when there are multiple catalogs and scripting is ON.
        var el = document.createElement('span');
        el.innerHTML = "<input type='hidden' id='catalog' name='catalog' />";
        var frm = $('#frmsrch').get(0);
        frm.appendChild(el);
    }

    // Initialize query box
    var el = $('#gsfx_bsrch_query').get(0);
    if ($.trim($(el).val()) !== "") {
        tval = $.trim($(el).val());
    } else {
        tval = fetchcookieval("lquery");
    }
    if (tval && tval !== '' && tval !== 'blank') {
        el.value = unescape(UnicodeFixup(jQuery.trim(tval)));
        setKeyBit(el);
    }
    else {
        // initialize the search box for first-time use
        $(el).bind("keypress paste", setKeyBit);
    }

    if (avlCatalogs.length == 0)
        return;

    // init the catalog
    var cats = $('#gsfx_bsrch_catsel a');
    tval = fetchcookieval("adcatalog");
    if (tval) {
        // if catalog cookie is set - click the catalog
        tval = unescape(tval);
        var i = 0
        for (; i < avlCatalogs.length; i++) {
            if (avlCatalogs[i].Value == tval) {
                $('#gsfx_cat_sel_div' + i).click();
                break;
            }
        }

        if (i == avlCatalogs.length) {
            ChangeCatSel(0);
        }
    }
    else {
        ChangeCatSel(0);
    }
}
function SaveSimpleSearch(url) {
    if (searchSubmit == 1) {
        return false;
    }

    // save Query Value
    var el = $("#gsfx_bsrch_query");
    submitSimpleSearch(url, el.val());
    return false;
}

//Advanced Search
function InitASrch() {

    $("#SearchDetails").css("display", "block");
    $("#divshowhide").css("display", "block");

    $("#showhide").toggle(function () {
        $("#divshowhide").attr("class", "hideMore");
        $("#showhide > span").html(hideoptions);
        $("#arrowimg").attr("src", arrowImage["up"]);
        $(".catalogRadioList").css("margin-bottom", "35px");
        $(".catalogRadioList").show();
        return false;
    }, function () {
        $("#divshowhide").attr("class", "showMore");
        $("#showhide  > span").html(showoptions);
        $("#arrowimg").attr("src", arrowImage["down"]);
        $(".catalogRadioList").css("margin-bottom", "0px");
        $(".catalogRadioList").hide();
        return false;
    });


    var el,
        f = $('#frmaSrch').get(0),
        qstr = (queryString['query']) ? queryString['query'] : '',
        astStr = (queryString['ast'] !== undefined) ? queryString['ast'] : '';
    // Initalize query box
    el = f.query;
    $(el).bind("keydown", function (event) {
        if (event.which === 13) {
            $('#btnSubmit').trigger("click");
        }
    })
    if (qstr == '') {
        var tval = fetchcookieval("lquery");
        if (tval) {
            el.value = unescape(UnicodeFixup(jQuery.trim(tval)));
            setKeyBit(el);
        }
        else {
            $(el).bind("keypress paste", setKeyBit);
        }
    }

    // Init Catalog Element
    multicatalog = false;
    el = f.catalog;
    if (el != null) {
        if (el.length && el.type != "select-one") {
            InitRadio(el, "catalog");
            multicatalog = true;
        }
        else {
            InitSelect(el, "catalog", "ad");
        }

        var alreadyChecked = false;
        for (i = 0; i < el.length; i++) {
            if (el[i].checked) { alreadyChecked = true; }
        }

        if (!alreadyChecked) {
            el[0].checked = true;
        }
    }

    var optcookie = new OptionCookie();
    // Init Scope Options    
    // if "ast" is presented on query string, then ignore the values saved in cookie
    if (astStr == "") {

        el = document.getElementsByName("ast");
        for (j = 0; j < el.length; j++) {
            if (el[j].disabled) {
                continue;
            }
            tval = optcookie["ad" + el[j].value];
            if (tval) {
                el[j].checked = (tval === '0' ? false : true);
            }
        }
    }
    else {
        var astArr = astStr.split(',');
        $('[name=ast]').each(function () {
            if (this.disabled === false) {
                $(this).attr("checked", ($.inArray(this.value, astArr) > -1 ? true : false));
            }
        });
    }
    if (optElems) {
        opt = optElems.split('|');
        for (j = 0; j < opt.length; j++) {
            el = $('#' + opt[j].toString()).get(0);
            if (el) {
                tval = optcookie["ad" + opt[j]];
                if (tval) {
                    if (tval == '0') el.checked = false;
                    else el.checked = true;
                }
            }
        }
    }
}

function InitSelect(elem, name, prefix, isScanned) {
    if (elem != null) {
        tval = fetchcookieval(prefix + name);

        if (name == "SPID") {
            if (!tval) tval = "global";
        }

        if (tval) {
            for (i = 0; i < elem.options.length; i++) {
                if (elem.options[i].value == unescape(tval)) {
                    elem.selectedIndex = i;
                    break;
                }
            }
        }
    }
}

/*
Select radio option
*/
function SelectRadio(elem, tval) {
    if (tval) {
        for (i = 0; i < elem.length; i++) {
            if (elem[i].value == unescape(tval)) {
                elem[i].checked = true;
                elem[i].click();
                break;
            }
        }
    }
}

/*
Initialize Radio option if it is not disabled
*/
function InitRadioEx(elem, name) {
    tval = fetchcookieval("ad" + name);
    if (tval) {
        for (i = 0; i < elem.length; i++) {
            if (elem[i].value == unescape(tval)) {
                if (!elem[i].disabled) {
                    elem[i].checked = true;
                    elem[i].click();
                    break;
                }
            }
        }
    }
}

/*
Disables all elements inside it
*/
function disableAll(elem) {
    if (elem == null || elem == 'undefined' || elem.tagName == null || elem.tagName == 'undefined')
        return;
    elem.disabled = true;
    for (var i = 0; i < elem.childNodes.length; i++) {
        disableAll(elem.childNodes[i]);
    }
}

/*
Enables all elements inside it
*/
function enableAll(elem) {
    if (elem == null || elem == 'undefined' || elem.tagName == null || elem.tagName == 'undefined')
        return;
    elem.disabled = false;
    for (var i = 0; i < elem.childNodes.length; i++) {
        enableAll(elem.childNodes[i]);
    }
}

function SearchLiveCatalog(frm, catalog) {
    if (catalog == 1)
        StatsDotNet.OptionCollectionId = SetLogCollectionBit(StatsDotNet.OptionCollectionId, 53);

    if (catalog == 2)
        StatsDotNet.OptionCollectionId = SetLogCollectionBit(StatsDotNet.OptionCollectionId, 54);

    $('#lsc').get(0).value = catalog;
    if (catalog == 2) {
        //save catalog in cookie
        var catalogList = $('#catalog').get(0);
        if (catalogList && catalogList.options) {
            var pvalue = catalogList.options[catalogList.selectedIndex].value;
            if (pvalue != "") {
                srch_setcookieval('adcatalog', pvalue);
            }
        }

        var msurl, qry;
        var elem = $('#frmaSrch').get(0).query;
        qry = $('#frmaSrch').get(0).query.value;

        qry = jQuery.trim(qry);
        if (window.encodeURIComponent) { qry = encodeURIComponent(qry); }
        else { qry = OutputEncoder_EncodeUrl(qry); }

        msurl = msnurl + qry;
        document.location.href = msurl;
        return false;
    }
    SubmitSearch(frm);
    $('#frmaSrch').get(0).submit();
}

function gsfx_bsrch_changeCatSelection(index) {
    ChangeCatSel(index);

    srch_setcookieval('adcatalog', escape(avlCatalogs[index].Value));
}


function ChangeCatSel(index) {
    $('#catalog').get(0).value = avlCatalogs[index].Value;
    if (index == currentCatSelIndx)
        return;

    $('#gsfx_cat_sel_div' + index).css({ 'font-weight': 'bold', 'cursor': 'default' });
    $('#gsfx_cat_sel_img' + index).css({ 'display': 'block' });

    $('#gsfx_cat_sel_div' + currentCatSelIndx).css({ 'font-weight': 'normal', 'cursor': 'pointer' });
    $('#gsfx_cat_sel_img' + currentCatSelIndx).css({ 'display': 'none' });

    currentCatSelIndx = index;
}

function selectDropDownItem(ddl, value, func) {
    for (var ii = 0; ii < ddl.length; ii++) {
        if (ddl[ii].value == value) {
            ddl[ii].selected = true;
            if (func != null)
                func(ddl);
            return ii;
        }
    }
    return -1;
}

// this event is fired when a Catalog radio button is clicked:
function CatalogOption_click(src, enableScopeOptions) {
    try {
        // get all checkboxes
        var catalogOptions = $('input');
        for (var i = 0; i < catalogOptions.length; i++) {
            var parentid = catalogOptions[i].attributes["_parentid"];
            // check only elements with parentid defined    
            if ((parentid != null) && (parentid != 'undefined') && (parentid != '')) {
                if (parentid.value == src.id) {
                    // catalog item is a child element of the catalog option which was clicked - enable it
                    if (src.checked) {
                        catalogOptions[i].disabled = false;
                    }
                    else {
                        catalogOptions[i].disabled = true;
                    }
                    var hrefs = catalogOptions[i].parentNode.getElementsByTagName('a');
                    for (var index = 0; index < hrefs.length; index++)
                        enable_link(hrefs[index]);
                }
                else {
                    // catalog item is not a child element of catalog option which was clicked - disable it
                    catalogOptions[i].disabled = true;
                    var hrefs = catalogOptions[i].parentNode.getElementsByTagName('a');
                    for (var index = 0; index < hrefs.length; index++)
                        disable_link(hrefs[index]);
                }
            }
        }
    }
    catch (e)
    { }
}

// note: these two functions handle the enabling/disabling of <a href=''> elements (links)
// the reason for this workaround is that the "disabled" attribute is not part of the <A> element in the HTML specs.
function disable_link(elem) {
    if ($(elem).is(":visible")) {
        var $linkSpan = $(elem).next("span[name=span_placeholder]");
        if ($linkSpan.length) {
            $linkSpan.show();
        }
        else {
            $(elem).after("<span name=\"span_placeholder\" style=\"color:gray\">" + $.trim($(elem).html()) + "</span>");
        }
        $(elem).hide();
    }
}
function enable_link(elem) {
    if ($(elem).is(":hidden")) {
        $(elem).next("span[name=span_placeholder]").hide();
        $(elem).show();
    }
}

function SaveRadioState(elem, name) {
    for (i = 0; i < elem.length; i++) {
        if (elem[i].checked) { srch_setcookieval('ad' + name, escape(elem[i].value)); }
    }
}

function Site(name, url, selected, clickAction) {
    var retVal =
    {
        Name: name,
        Url: url,
        Selected: selected,
        ClickAction: clickAction
    };
    return retVal;
}

var ClickAction = {
    Redirect: 0,
    AjaxCall: 1,
    FormSubmit: 2
}

function Catalog(name, value, selected) {
    var retVal =
    {
        Name: name,
        Value: value,
        Selected: selected
    };
    return retVal;
}
var gsfx_bsrch_InitCatSelection = function (targetid, charstr) {
    var cval = unescape(fetchcookieval('adcatalog'));
    var a = 0;
    var highlight = false;
    if (cval) {
        var catcon = $('#gsfx_bsrch_catsel').get(0);
        if (catcon) {
            for (var i = 0; i < catcon.childNodes.length; i++) {
                var el = catcon.childNodes[i];
                if (el && el.tagName && el.getAttribute('catalog')) {
                    if (el.getAttribute('catalog') == cval) {
                        el.className += ' gsfx_bsrch_highlight';
                        try {
                            MS.Support.AC.ACChangeCharStart(targetid, charstr.split(':')[a]);
                            MS.Support.AC.ACSetLcid(targetid, cval.split('=')[1]);
                        } catch (e) { }
                        highlight = true;
                    }
                    else {
                        el.className = el.className.replace(/( ?|^)gsfx_bsrch_highlight\b/gi, '');
                    }

                    a++;
                }
            }

            if (!highlight) {
                for (var i = 0; i < catcon.childNodes.length; i++) {
                    var el = catcon.childNodes[i];
                    if (el && el.tagName && el.getAttribute('catalog')) {
                        el.className += ' gsfx_bsrch_highlight';
                        return;
                    }
                }
            }
        }
    }
}

function AddGpsSearchToSearchOptions(siteName, siteUrl) {
    var isAdded = false;
    $(supportedSites).each(function (index) {
        if (this.Url === siteUrl) {
            isAdded = true;
            this.Name = siteName;
        }
    });
    if (!isAdded) {
        $(supportedSites).each(function (index) {
            this.Selected = false;
        });
        supportedSites = $.merge([new Site(siteName, siteUrl, true, ClickAction.AjaxCall)], supportedSites);
        currentSiteIndex = 0;
    }
    var optSubContainer = InitialSeachOptions();
    $("#gsfx_bsrch_options_subcntr").replaceWith(optSubContainer);
}

function removeGpsSearchFromSearchOptions(siteUrl) {
    var removed = false;
    var selected = false;
    supportedSites = $.map(supportedSites, function (item) {
        if (item.Url === siteUrl) {
            removed = true;
            return;
        } else {
            if (item.Selected) {
                selected = true;
            }
            return item;
        }
    });
    if (removed) {
        if (!selected) {
            supportedSites[0].Selected = true;
        }
        var optSubContainer = InitialSeachOptions();
        $("#gsfx_bsrch_options_subcntr").replaceWith(optSubContainer);
    }
}


function InitialSeachOptions() {
    var optSubContainer = document.createElement('DIV');
    optSubContainer.id = 'gsfx_bsrch_options_subcntr';

    var optCol_temp = document.createElement('DIV');

    var optCol2_temp = document.createElement('DIV');
    optCol2_temp.id = 'gsfx_srchsitename_div';
    optCol2_temp.className = 'gsfx_srchsitename_cotnr';

    for (var i = 0; i < supportedSites.length; i++) {
        var optCol2 = optCol2_temp.cloneNode(true);
        optCol2.id += i;
        optCol2.innerHTML = supportedSites[i].Name;

        var opt = optCol_temp.cloneNode(true);
        var localIndex = i;
        $(opt).bind("click", { index: localIndex }, ChangeSiteSelection);
        if (supportedSites[i].Selected) {
            $(opt).attr("class", "TopSearchOptionsSelected");
            ChangeFormAction(i);
        } else {
            $(opt).attr("class", "TopSearchOptionsDefault");
        }

        opt.appendChild(optCol2);
        optSubContainer.appendChild(opt);
    }
    $(optSubContainer).width($('#gsfx_bsrch_divQuery').width());
    return optSubContainer;
}

function CreateSearchOptions(id, acWidth) {
    optContainer = document.createElement('DIV');
    optContainer.id = 'gsfx_bsrch_options';

    var optSubContainer = InitialSeachOptions();
    optContainer.appendChild(optSubContainer);

    var isRTL = $('html').css('direction') == 'rtl';

    var ie6BgDiv = $('#gsfx_bsrch_bg_ie6').get(0);
    isIE6 = $('#gsfx_bsrch_bg_ie6').length > 0;
    if (isIE6) {
        if (isRTL) {
            ie6BgDiv.style.left = $('#gsfx_bsrch_bg').get(0).offsetLeft + 4;
        }
        else {
            ie6BgDiv.style.left = $('#gsfx_bsrch_bg').get(0).offsetLeft - 2;
        }
        ie6BgDiv.style.display = 'block';
    }

    var acDiv = $('#' + id).get(0);
    if (acDiv) {
        acDiv.appendChild(optContainer);
        $('#' + id + ' > :first-child').css(
                            {
                                'border-bottom': '0px',
                                'border-color': '#999999',
                                'margin-top': '0px'
                            });

        if (isRTL) {
            $('#' + id + ' > :first-child').css(
                            {
                                'margin-right': '-1px'
                            });
        }

        $('#' + id + ' > :first-child').css({ 'background-color': '#ffffff' });
        $('#' + id).css({ 'margin-left': '-1px', 'height': '0px' });

        var acObj = MS.Support.AC ? MS.Support.AC.ACArrayEl('gsfx_bsrch_query') : null;
        if (acObj) {
            acObj.options.handleResize = ResizeSiteOpts;
            acObj.options.acMinWidth = acWidth;
        }
    }
    else {
        $(optContainer).insertBefore('#gsfx_bsrch_query');
        if (isRTL) {
            $(optContainer).css({ 'position': 'relative', 'top': $('#gsfx_bsrch_divQuery').get(0).offsetHeight + 'px', 'float': 'right' });
        }
        else {
            $(optContainer).css({ 'top': $('#gsfx_bsrch_divQuery').get(0).offsetHeight + 'px', 'left': '-1px' });
        }
    }
    $('#gsfx_bsrch_query').bind('focus', function (event) { HandleQueryFocus(event); $('#acListWrappergsfx_bsrch_query').show(); });
    $('#gsfx_bsrch_btnimg').live('focus', function (event) {
        $('#acListWrappergsfx_bsrch_query').show();
        $('#gsfx_bsrch_options').show();
        $('#gsfx_bsrch_options_subcntr :eq(currentSiteIndex)').show();
    });
    $('#gsfx_bsrch_btnimg').live('blur', function (event) {
        $('#acListWrappergsfx_bsrch_query').hide();
    });
    $('#gsfx_bsrch_query').bind('click.srchsiteopts', function (event) { HandleQueryFocus(event); $('#gsfx_bsrch_query').unbind('click.srchsiteopts'); });
    $('#gsfx_bsrch_query').bind('keydown.srchsiteopts', function (event) {
        HandleKeydown(event);
        if ($('#acListWrappergsfx_bsrch_query :first-child').css("visibility") === "hidden") {
            var index = -1;
            if (event.which == 40) {
                index = currentSiteIndex + 1;
                if (index >= supportedSites.length) {
                    return;
                }
            }
            else if (event.which == 38) {
                index = currentSiteIndex - 1;
                if (index < 0) {
                    return;
                }
            }
            if (index != -1) {
                $('#gsfx_srchsitename_div' + index).parent().attr("class", "TopSearchOptionsSelected");
                $('#gsfx_srchsitename_div' + currentSiteIndex).parent().attr("class", "TopSearchOptionsDefault")
                supportedSites[currentSiteIndex].Selected = false;

                supportedSites[index].Selected = true;
                currentSiteIndex = index;
            }
        }
        if (event.which === 13) {
            return SaveSimpleSearch();
        }
    });
    $(document).bind('click.srchsiteopts', function (event) { SimpleSearchClickHandler(event); });
}

function SimpleSearchClickHandler(event) {
    if (!event || !event.target) {
        return;
    }
    var targetEl = event.target;

    if ($(targetEl).hasClass('gsfx_img_png')) {
        return;
    }
    var searchContainer = $('#gsfx_bsrch_divQuery').get(0);

    var isSearchElement = (targetEl == searchContainer);

    if (!isSearchElement)
        isSearchElement = IsChild(targetEl, searchContainer);

    if (isSearchElement) {
        HideLangSel();
        $(optContainer).css({ 'display': 'block' });
    }
    else {
        $(optContainer).css({ 'display': 'none' });
    }

    if (avlCatalogs.length == 0) {
        return;
    }

    if (targetEl == $('#gsfx_cat_sel').get(0)) {
        return;
    }

    var isCatSelChild = IsChild(targetEl, $('#gsfx_cat_sel').get(0), 3);

    if (!isCatSelChild) {
        HideLangSel();
    }
}

function IsChild(targetEl, parentEl, nestingLimit) {
    if (!targetEl) {
        return false;
    }

    var isChildElement = false;
    if (!nestingLimit)
        nestingLimit = 6;

    var parent = targetEl.parentNode;
    var level = 0;
    var isSearchElement = false;
    while (parent && nestingLimit > level) {
        if (parent == parentEl) {
            isChildElement = true;
            break;
        }
        parent = $(parent).parent().get(0);
        level++;
    }
    return isChildElement;
}

function ResizeSiteOpts(listDiv) {
    if (listDiv.style.visibility != 'hidden' && listDiv.childNodes.length > 0) {
        optContainer.style.top = $(listDiv).get(0).clientHeight + "px";
    }
    else {
        optContainer.style.top = 0;
    }

    if (listDiv.style.visibility != 'hidden')
        $(optContainer).css({ 'display': 'block' });
    else if ($(document).get(0).activeElement != $('#gsfx_bsrch_query').get(0))
        $(optContainer).css({ 'display': 'none' });
}

function HandleKeydown(event) {
    if (event.keyCode == 9) {
        if ($(optContainer).prev().get(0) == null || $(optContainer).prev().css('visibility') == 'hidden') {
            $(optContainer).css({ 'display': 'none' });
            return;
        }
    }

    if (event.keyCode == 13) return;

    HandleQueryFocus(event);
}

function HandleQueryFocus(event) {
    if (event.type == 'focus' && prevElement == event.target) {
        return;
    }
    HideLangSel();
    $(optContainer).css({ 'display': 'block' });
}
var ChangeFormAction = function (index) {
    $("#frmsrch").attr("action", supportedSites[index].Url);
}

function ChangeSiteSelection(event) {
    var index = event.data.index;
    ChangeFormAction(index);
    if (supportedSites[index].Selected) {
        return;
    }

    $('#gsfx_srchsitename_div' + index).parent().attr("class", "TopSearchOptionsSelected");

    $('#gsfx_srchsitename_div' + currentSiteIndex).parent().attr("class", "TopSearchOptionsDefault");
    supportedSites[currentSiteIndex].Selected = false;

    supportedSites[index].Selected = true;
    currentSiteIndex = index;
}

function InitCatLangSel() {
    var $ct = $('#gsfx_cat_sel'),
        $btn = $('#gsfx_cat_sel_btn'),
        $btnImg = $('#gsfx_cat_sel_btn img')
    $panel = $('#gsfx_cat_sel_cntnr');

    if ($btn.length === 0 || $panel.length === 0) {
        return;
    }
    if (avlCatalogs && avlCatalogs.length <= 1) {
        $container.hide();
        return;
    }
    $btn.click(function (e) {
        $panel.css("top", $btnImg[0].height + "px");
        $(optContainer).hide();
        $panel.toggle();
        SimpleSearchClickHandler(e);    // call necessary handler manually due to returning false in this handler to stop event bubbling.
    });
    if ($btnImg.attr('hover-src')) {
        $btnImg.attr('normal-src', $btnImg.attr('src'));
        $btn.hover(function () {
            $btnImg.attr("src", $btnImg.attr('hover-src'));
        }, function () {
            $btnImg.attr("src", $btnImg.attr('normal-src'));
        });
    }

    $panel.width(($('#gsfx_bsrch_query').parent().width() / 2) + $btn.width() + 35);
    $ct.show();
}

function HideLangSel() {
    $panel = $('#gsfx_cat_sel_cntnr').hide();
}

function CallWebSearch(searchHomeUrl, searchResultsUrl) {
    var f = $('#frmaSrch').get(0);

    // save Query Value
    var el = f.query;

    if ($.trim($(el).val()).length == 0) {
        StatsDotNet.OptionCollectionId = SetLogCollectionBit(StatsDotNet.OptionCollectionId, 33);
        StatsDotNet.targetUrl = searchHomeUrl;
        document.location.href = searchHomeUrl;
        return;
    }

    SaveSimpleSearch(searchResultsUrl);
}
