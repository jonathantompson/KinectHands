if (!(window.MS && MS.Support && MS.Support.Fms))
{
	throw "Missing namespace MS.Support.Fms. \ngsfx/gsfxsurvey.js should be rendered after survey.js.";
}

if (!MS.Support.Fms.Gsfx)
{
	MS.Support.Fms.Gsfx = {};
}

if (!MS.Support.Fms.Gsfx.GsfxSurvey)
{
	MS.Support.Fms.Gsfx.GsfxSurvey = function (survey)
	{
		//using
		var Fms = MS.Support.Fms;
		var Survey = Fms.Survey;
		var Utils = Fms.Utils;
		var addEventHandler = Utils.addEventHandler;
		var removeEventHandler = Utils.removeEventHandler;

		var QuitMode =
		{
			"giveup": 1,
			"cancel": 2,
			"persist": 0
		};

		function getQuitModeByAction(action)
		{
			var quitMode = QuitMode[action.toLowerCase()];

			if (quitMode == null)
			{
				throw "Unknown action: " + action;
			}

			return quitMode;
		}

		function createHiddenFrame(name)
		{
			var container = document.createElement("DIV");
			container.innerHTML = "<IFRAME id=\"" + name + "\" name=\"" + name + "\" src=\"about:blank\" border=0 frameBorder=0 style=\"width:0;height:0\"></IFRAME>";
			(document.body || document.documentElement).appendChild(container);

			return document.getElementById(name);
		}

		function createSubmitForm(target)
		{
			var submitForm = document.createElement("form");
			submitForm.method = "POST";
			submitForm.encoding = "application/x-www-form-urlencoded";
			submitForm.enctype = "application/x-www-form-urlencoded";
			submitForm.action = "https://" + window.location.hostname + (window.location.port ? (":" + window.location.port) : "") + "/common/survey.aspx";
			submitForm.target = target;

			(document.body || document.documentElement).appendChild(submitForm);

			return submitForm;
		}

		function postback(survey, action)
		{
			if (survey.isThankyouPage)
			{
				return;
			}

			var quitMode = getQuitModeByAction(action);

			if (survey.suppressSubmission)
			{
				if (quitMode == 0)
				{
					if (survey.thankyou != null)
					{
						survey.thankyou.show();
					}
				}
				return true;
			}

			if (surveyStyle == "embedded")
			{
				action = "FINISHEMBED:" + survey.submitFields["FMSURL"];
			}

			survey.addSubmitField("FMSACTION", action);

			if (survey.parameters)
			{
				survey.addSubmitField("PARAMLENGTH", survey.parameters.length);

				for (var i = 0; i < survey.parameters.length; ++i)
				{
					if (typeof (survey.parameters[i]) != "undefined")
					{
						survey.parameters[i] = encodeURIComponent(survey.parameters[i]);
					}
				}

				survey.addSubmitField("PARAMS", survey.parameters.join(","));
			}

			var surveyAnswers = survey.encodeAnswers(function (input) { return unicodeFixup(escape(input)); });
			survey.addSubmitField("DATALENGTH", surveyAnswers.split("|").length);
			survey.addSubmitField("SURVEYANSWERS", surveyAnswers + "|" + quitMode);

			if (isKBEmbedded)
			{
				survey.addSubmitField("CONTENTTYPE", StatsDotNet.contentType || "");
				survey.addSubmitField("CONTENTCULTURE", StatsDotNet.ContentCulture || "");
				survey.addSubmitField("CONTENTID", StatsDotNet.contentId || "");
				survey.addSubmitField("CONTENTLCID", StatsDotNet.contentLn || "");
				survey.addSubmitField("SITECULTURE", StatsDotNet.SiteCulture || "");
				survey.addSubmitField("SSID", StatsDotNet.ssId || "");
				survey.addSubmitField("SITEBRANDID", StatsDotNet.siteBrandId || "");
				survey.addSubmitField("SSVERSION", StatsDotNet.SsVersion || "");
				survey.addSubmitField("CONTENTPROPERTIES", StatsDotNet.ContentProperties || "");
			}

			var target = "submitframe_" + survey.id;
			var submitFrame = createHiddenFrame(target);
			var submitForm = createSubmitForm(target);

			for (var field in survey.submitFields)
			{
				var fieldElement = document.createElement("input");
				fieldElement.type = "hidden";
				fieldElement.name = field;
				fieldElement.value = survey.submitFields[field];

				submitForm.appendChild(fieldElement);
			}

			if (survey.isInvitation || surveyStyle == "embedded")
			{
				submitForm.target = "_self";
				try
				{
					submitForm.submit();
					delayHalfSecond();
				}
				catch (ex)
				{
				}
				return;
			}
			else
			{
				if (quitMode == QuitMode.persist)
				{
					if (isKBEmbedded)
					{
						setKBVisited();
					}
				}

				try
				{
					setTimeout(function ()
					{
						submitForm.submit();
					}, 0);

					if (quitMode == QuitMode.persist && survey.thankyou != null)
					{
						removeEventHandler(document, "keypress", handleKeypress);
						survey.thankyou.show();
					}

					if (quitMode == QuitMode.giveup)
					{
						delayHalfSecond(1500);
					}
				}
				catch (e)
				{
				}

				if (survey.isStandalone && (!survey.isInvitation) && surveyStyle != "embedded" && quitMode == QuitMode.cancel)
				{
					window.top.close();
					location.reload();
				}
			}
		}

		function handleInvitationSubmit(survey, action)
		{
			var surveyDiv;

			var isPersist = ((action == "persist") ? true : false);

			if (!isPersist)
			{
				postback(survey, "cancel");
			}

			try
			{
				surveyDiv = window.parent.document.getElementById('surveyDivBlock');
				surveyDiv.style.display = 'none';
				surveyDiv.style.height = 0;
				surveyDiv.firstChild.style.height = 0;
			}
			catch (e) { }

			if (isPersist)
			{
				if (survey.isPreview)
				{
					window.location.href = Replace(window.location.href, "surveyinvite.aspx", "survey.aspx");
					return;
				}
				else if (survey.isOnUnloadSurvey)
				{
					var surveywin = window.top.open(Replace(window.location.href, "surveyinvite.aspx", "survey.aspx"), "_blank", "resizable=1,left=200,top=200,width=1024,height=700,scrollbars=1,status=1");
					if (surveywin)
					{
						try
						{
							// for Netscape display like IE, surveywin will always open in new tab and surveywin.blur() will hide the whole window
							if (!(window.navigator.userAgent.indexOf("Netscape") > 0 && window.navigator.userAgent.indexOf("MSIE") > 0))
							{
								surveywin.blur();
							}

							// for Netscape display like Firefox, the follow statement may cause current function been terminated without any prompt and statements below this will never been executed
							surveywin.openerWinLocation = window.top.location.href;
						}
						catch (e) { };
					}
				}
				else
				{
					if (survey.surveyRedirect)
					{
						window.open(survey.surveyRedirect);
					}
					else
					{
						window.open(Replace(window.location.href, "surveyinvite.aspx", "survey.aspx"), "_blank");
					}
				}
			}
			else if (surveyDiv != null && action == "cancel")
			{
				addEventHandler(
					window,
					"unload",
					function ()
					{
						try
						{
							if (window.navigator.userAgent.indexOf("MSIE") > -1)
							{
								window.parent.setTimeout("var surveyDiv = document.getElementById(\"surveyDivBlock\");surveyDiv.removeChild(surveyDiv.firstChild);", 1000);
							}
							else if (window.navigator.userAgent.indexOf("Firefox") > -1)
							{
								surveyDiv.removeChild(surveyDiv.firstChild);
							}
							document.open();
							document.close();
						}
						catch (e) { }
					}
				);
			}
			else
			{
				delayHalfSecond(1500);
			}
		}

		function setKBVisited()
		{
			var count = 0;

			for (var i = 0; i < survey.kbvisited.length; i++)
			{
				if (survey.kbvisited.charAt(i) == '|')
					count++;
			}
			if (count >= survey.maxKBsInCookie)
			{
				var index = survey.kbvisited.indexOf("|", 1);
				survey.kbvisited = survey.kbvisited.substring(index);
			}
			survey.kbvisited = survey.kbvisited + "|" + kbSurvey.submitFields["SURVEYSCID"].replace(/;/g, ":") + "@" + g_currentContent;
			Utils.setSessionCookie("kbvisited", survey.kbvisited);
		}

		this.submit = function (survey, action)
		{
			if (isOnUnloadSurvey && isTracking)
			{
				setDomainIsTracking(false);
				isTracking = false;
			}

			if (survey.submitted)
			{
				return;
			}
			survey.submitted = true;

			if (survey.isInvitation)
			{
				handleInvitationSubmit(survey, action);
			}
			else
			{
				postback(survey, action);
			}
		}

		survey.submitHandler = this.submit;

		var surveyStyle = survey.surveyStyle;

		var openerWin = null;
		var openerUrl = null;
		var isOnUnloadSurvey = survey.isOnUnloadSurvey ? true : false;
		var urlHashCleaner = /#.*$/;

		var isTracking = false;

		var triggerContains = survey.surveyTriggerContains;
		var triggerPages = survey.surveyTriggerPages;

		var isKBEmbedded = survey.isKBEmbedded ? true : false;

		function hyperLinkOnClick(e)
		{
			if (e == 'undefined')
			{
				e = window.event;
			}
			var a = e.srcElement ? e.srcElement : e.target;
			while (a && a.tagName && a.tagName.toLowerCase() != "a")
			{
				a = a.parentNode;
			}
			if (a && a.href != null)
			{
				if (a.href.toLowerCase().indexOf("javascript:") < 0)
				{
					a.target = "_blank";
				}
				else
				{
					return false; // bug 61209
				}
			}
		}

		function handleKeypress(e)
		{
			
			if (isTracking)
			{
				return;
			}

			if (typeof (e) == 'undefined')
			{
				return;
			}
			
			if (survey.suppressKeypressHandler)
			{
				return;
			}

			if (e.keyCode == 27 && surveyStyle == "embedded")
			{
				survey.cancel();

				if (e.preventDefault)
				{
					e.preventDefault();
				}
				else
				{
					e.returnValue = false;
				}
			}
			if (e.keyCode == 13)
			{
				var source = e.srcElement ? e.srcElement : e.target;
				if (source == null || (source.tagName.toUpperCase() != "A" && source.type != "textarea" && source.type != "button"))
				{
					if ((survey.thankyou != null && survey.getCurrentPage() == survey.thankyou && survey.thankyou.isVisible()) || survey.isShowAll)
					{
						window.top.close();
					}
					else
					{
						survey.next();
					}

					if (e.preventDefault)
					{
						e.preventDefault(); //w3c style
					}
					else
					{
						e.returnValue = false; // IE
					}
				}
			}
		}

		function isKBVisited()
		{
			survey.kbvisited = Utils.getCookie("kbvisited");
			if (g_currentContent == null || g_currentContent == "")
				return;
			g_currentContent = g_currentContent.replace(";", ":");
			g_currentContent = g_currentContent.replace(";", ":");
			if (survey.kbvisited == null || survey.kbvisited.charAt(0) != '|')
				survey.kbvisited = "";
			var index = survey.kbvisited.indexOf(g_currentContent);

			return index >= 0;
		}

		function validateErrorHandler(survey, validateResult)
		{
			alert(validateResult.errorMessage);
			return false;
		}

		function handleWindowBeforeUnload()
		{
			// bug 59632
			// Netscape 7.2 will fire onbeforeunload event will change location.hash
			// we use window.showsurveymutex to prevent submit while set  hash to '#showsurvey'
			if (typeof (window.showsurveymutex) != "undefined" && window.showsurveymutex == true)
			{
				return;
			}
			survey.giveup();
		}

		function handleSectionChanged()
		{
			if (survey.isStandalone && window.scrollTo)
			{
				var documentElement = document.body ? document.body : document.documentElement;
				if (documentElement.currentStyle && documentElement.currentStyle.blockDirection == "rtl")
				{
					window.scrollTo(documentElement.scrollWidth, 0);
				}
				else
				{
					window.scrollTo(0, 0);
				}
			}

			return true;
		}

		function prepareHttpsTunnel()
		{
			var img = document.createElement("img");
			img.src = "https://" + window.location.hostname + (window.location.port ? (":" + window.location.port) : "") + "/library/images/support/cn/onepix.gif";
			img.width = 0;
			img.height = 0;
			(document.body || document.documentElement).appendChild(img);
		}

		this.start = function ()
		{
			if (survey.isStandalone)
			{
				document.title = survey.name;
			}

			if (survey.isShowAll)
			{
				survey.suppressSubmission = true;
				survey.displayBranchRules();
			}

			if (!isKBEmbedded)
			{
				addEventHandler(window, "beforeunload", handleWindowBeforeUnload);
				prepareHttpsTunnel();
			}

			if (survey.thankyou != null && window.opener == null)
			{
				var userAgent = navigator.userAgent;
				if (userAgent.indexOf("compatible") == -1 || userAgent.indexOf("MSIE") == -1)
				{
					var closeButton = Utils.getChildById(survey.thankyou.domObject, "SurveyCloseButton");
					if (closeButton)
					{
						closeButton.style.display = "none";
					}
				}
			}

			if (survey.isInvitation)
			{
				return true;
			}

			if ((!survey.isShowAll) && !(window.navigator.userAgent.indexOf("Netscape/8.1") > -1 && window.navigator.userAgent.indexOf("MSIE") > -1)) // skip Netscape 8.1 (display like IE), bug 59414
			{
				if (surveyStyle == "full screen")
				{
					try
					{
						window.moveTo(0, 0);
						window.resizeTo(screen.availWidth, screen.availHeight);
					}
					catch (e)
					{
						// move or resize a window by script while user dragging the window may cause an Access Denied exception
						// see KB http://support.microsoft.com/kb/904947
					}
				}
				else
				{
					// If surveystyle is specified in query string, and equal popup, then resize window also
					if (navigator.userAgent.indexOf("Firefox") < 0) // skip Firefox, it resizes the whole browser when opens in a new tab
					{
						if (window.top == window && (survey.pageWidth || survey.pageHeight) || (surveyStyle != null && surveyStyle.toLowerCase() == "popup"))
						{
							var width = survey.pageWidth || 1024;
							var height = survey.pageHeight || 700;
							try
							{
								resizeTo(width, height);
							}
							catch (e)
							{
								// move or resize a window by script while user dragging the window may cause an Access Denied exception
								// see KB http://support.microsoft.com/kb/904947
							}
						}
					}
				}
			}

			if (isKBEmbedded)
			{
				if (isKBVisited())
				{
					survey.suppressSubmission = true;

					for (var index = 0; index < survey.pages.length; ++index)
					{
						survey.pages[index].hide();
					}

					if (survey.thankyou != null)
					{
						survey.thankyou.show();
					}

					survey.show();

					return false;
				}
			}

			survey.onAfterNext.add(new Fms.SurveyEventDelegate(null, handleSectionChanged));
			survey.onAfterSkip.add(new Fms.SurveyEventDelegate(null, handleSectionChanged));
			survey.onAfterPrevious.add(new Fms.SurveyEventDelegate(null, handleSectionChanged));

			survey.onValidateError.add(new Fms.SurveyEventDelegate(null, validateErrorHandler));

			if (survey.isStandalone)
			{
				addEventHandler(document, "click", hyperLinkOnClick); // in order to open link in new window
				addEventHandler(document, "keypress", handleKeypress);

				if ((!survey.isInvitation) && (!survey.isShowAll) && isOnUnloadSurvey)
				{
					//(bug#58710)
					// the qualifications for start tracking are
					// 1. opened by window.open();  ==> window.opener not null
					// 2. current page is the first page of a window/tab (for IE or Netscape display like IE, window.history.length == 0, for other browser, window.history.length ==1)
					// 3. has not been shown before; ==> flag "#showsurvey" not exist
					if (window.opener != null
						&& (window.history.length == 0 || ((window.navigator.userAgent.indexOf("MSIE") < 0 || window.navigator.userAgent.indexOf("MSIE 10") > -1) && window.history.length == 1))
						&& (window.location.hash != "#showsurvey")
					)
					{
						try
						{
							openerWin = window.opener;
							if (typeof (window.openerWinLocation) != 'undefined')
							{
								openerUrl = window.openerWinLocation;
							}
							else
							{
								openerUrl = openerWin.location.href;
							}
							openerUrl = openerUrl.replace(urlHashCleaner, "");
							var surveyTrackingMsg = document.createElement("DIV");
							surveyTrackingMsg.id = "surveyTrackingMsg";
							surveyTrackingMsg.className = "TRACKINGMSG";
							surveyTrackingMsg.innerHTML = survey.getTrackingText();
							survey.domObject.parentNode.appendChild(surveyTrackingMsg);
							setDomainIsTracking(true);
							isTracking = true;
							startTracking();
						}
						catch (e)
						{
						}
					}
				}
			}

			if (!isTracking)
			{
				showHiddenSurvey();
				return true;
			}
			else
			{
				return false;
			}
		}

		function showHiddenSurvey()
		{
			var surveyTrackingMsg = document.getElementById("surveyTrackingMsg");
			if (surveyTrackingMsg)
			{
				surveyTrackingMsg.style.display = "none";
			}
			if (isOnUnloadSurvey && window.opener && (window.history.length == 0 || ((window.navigator.userAgent.indexOf("MSIE") < 0 || window.navigator.userAgent.indexOf("MSIE 10") > -1) && window.history.length == 1)))
			{
				// bug#59632 netscape 7.2 has a bug and will fire onbeforeunload event while change the location.hash but not really reload the page.
				// in this case, onbeforeunload will return back and continue executing immediately and unload event will not been fired anymore.
				// use this mutext to prevent submit in onbeforeunload
				window.showsurveymutex = true;
				// once a survey has been show, we append a hash "showsurvey"  to current location as a flag
				window.location.replace("#showsurvey");
				window.showsurveymutex = false;
			}

			if (survey.surveyRedirect)
			{
				window.location = survey.surveyRedirect;
			}
			else
			{
				survey.show();
			}
			window.focus();

			if (isOnUnloadSurvey && isTracking)
			{
				setDomainIsTracking(false);
				isTracking = false;
			}
		}

		function inTriggerPages(pages, url)
		{
			for (var i = 0; i < pages.length; i++)
			{
				if (wildcardMatch(url.toUpperCase(), pages[i].replace(urlHashCleaner, "").toUpperCase()))
				{
					return true;
				}
			}
			return false;
		}

		function getRelativePath(url)
		{
			var start = url.indexOf('//');
			var relativest1 = url.indexOf('/', start + 2);
			return url.substring(relativest1);
		}

		function startTracking()
		{
			try
			{
				if (openerWin.closed == false)
				{
					var openerLocation = openerWin.location.href.replace(urlHashCleaner, "");
					var pathName = getRelativePath(openerLocation);
					if (openerLocation == openerUrl || inTriggerPages(triggerPages, pathName) || triggerContains.toLowerCase() == "domain" || triggerContains.toLowerCase() == "sub-domain")
					{
						window.setTimeout(startTracking, 2000);
						return;
					}
				}
			}
			catch (e)
			{
			}

			if (triggerContains.toLowerCase() == "domain" || triggerContains.toLowerCase() == "sub-domain")
			{
				var lastbeat = getLastHeartBeat();
				var now = new Date().getTime();
				if (now - lastbeat < 3000)
				{
					window.setTimeout(startTracking, 2000);
					return;
				}
			}

			showHiddenSurvey();
			survey.start();
		}

		function Replace(strOrig, str1, str2)
		{
			if (strOrig.length == 0 || str1.length == 0)
				return strOrig;
			var index = 0; var indexend = 0; var len1 = str1.length;
			var result = "";
			do
			{
				indexend = strOrig.indexOf(str1, index);
				if (indexend == -1)
				{
					indexend = strOrig.length;
				}
				result += strOrig.substring(index, indexend);
				if (indexend != strOrig.length)
					result += str2;
				index = indexend + len1;
				if (index >= strOrig.length)
					break;
			}
			while (true);
			return result;
		}

		//check END WITH * wildcard if not exact match
		function wildcardMatch(source, pattern)
		{
			if (source == pattern)
			{
				return true;
			}

			if (pattern.length > 0)
			{
				if (source == pattern)
				{
					return true;
				}

				if (pattern.length > 0)
				{
					var body = trimEnd(pattern, "*");
					if (pattern.charAt(pattern.length - 1) == '*' && source.indexOf(body) == 0)
					{
						return true;
					}
				}
			}

			return false;
		}

		function trimEnd(str, ch)
		{
			var body = str;
			if (body.length)
			{
				while (body.length > 0 && body.charAt(body.length - 1) == ch)
				{
					body = body.substr(0, body.length - 1);
				}
			}
			return body;
		}

		function getLastHeartBeat()
		{
			var entry = Utils.getCookie("fmshb");
			if (entry)
			{
				try
				{
					return entry.split(',')[1];
				}
				catch (e)
				{
					return null;
				}
			}

			return null;
		}

		function isDomainTracking()
		{
			var entry = Utils.getCookie("fmshb");
			if (entry)
			{
				try
				{
					return entry.split(',')[0] == "1" ? true : false;
				}
				catch (e)
				{
				}
			}

			return false;
		}

		function setDomainIsTracking(value)
		{
			if (isDomainTracking() != value)
			{
				var flag = value ? '1' : '0';
				Utils.setSessionCookie("fmshb", flag + "," + (new Date().getTime()));
			}
		}

		function delayHalfSecond(delay)
		{
			try
			{
				if (!delay) delay = 500;
				var today = new Date();
				var now = today.getTime();
				while (1)
				{
					var today2 = new Date();
					var now2 = today2.getTime();
					if ((now2 - now) > delay) { break; };
				}
			}
			catch (e) { }
		}

		function unicodeFixup(s)
		{
			var result = new String();
			var c = '';
			var i = -1;
			var l = s.length;
			result = '';
			for (i = 0; i < l; i++)
			{
				c = s.substring(i, i + 1);
				if (c == '%')
				{
					result += c; i++;
					c = s.substring(i, i + 1);
					if (c != 'u')
					{
						if (parseInt('0x' + s.substring(i, i + 2)) > 128) { result += 'u00'; }
					}
				}
				/* Product Studio Bug 37129
				This fix is needed to preserve '+' in the input when client-side escaped strings are decoded in server-side code.
				Jscript escape() does not escape a '+' to '%2B'.
				System.Web.HttpUtility.UrlDecode() replaces '+' with a space, but decodes '%2B' just fine.
				Jscript unescape() also decodes '%2B' just fine. */
				else if (c == '+')
				{
					c = '%2B';
				}
				result += c;
			}
			return result;
		}
	}

	MS.Support.Fms.Gsfx.GsfxSurvey.SurveyStartHandler = function(sender, args)
	{
		var survey = sender;
		var gsfxSurvey = new MS.Support.Fms.Gsfx.GsfxSurvey(survey);

		survey.onBeforeStart.remove(new MS.Support.Fms.SurveyEventDelegate(null, arguments.callee));

		return gsfxSurvey.start();
	}

	if (MS.Support.Fms.Gsfx.DataCollection == null)
	{
		MS.Support.Fms.Gsfx.DataCollection = {};

		MS.Support.Fms.Gsfx.DataCollection.HandleDataCollectionLinkClick = function(e, defaultErrorMsg)
		{
			if (!defaultErrorMsg)
			{
				defaultErrorMsg = "Diagnostic Service is currently unavailable. Please try later.";
			}
			var cancelDefault = true;

			if (!MS.Support.Fms.Gsfx.DataCollection.HandleDataCollectionLinkClick.isInProgress)
			{
				MS.Support.Fms.Gsfx.DataCollection.HandleDataCollectionLinkClick.isInProgress = true;

				var link = e.srcElement || e.target;
				var survey = MS.Support.Fms.Survey.GetSurveyInstanceByElement(link);

				if (survey)
				{
					var currentQuestion = survey.getCurrentPage().getQuestionByElement(link);
				}

				if (currentQuestion && currentQuestion.clickedLinks && currentQuestion.clickedLinks["_id" + link.id])
				{
					var data = currentQuestion.clickedLinks["_id" + link.id];
					if (data && !data.hasError)
					{
						link.href = data.url;
						link.target = "_blank";
						cancelDefault = false;
					}
				}
				else
				{
					var xml = MS.Support.Fms.Gsfx.DataCollection.RequestManifestInstance();
					if (xml && xml.documentElement)
					{
						var r = xml.documentElement;
						Utils = MS.Support.Fms.Utils;

						var fatalError = Utils.findFirstChild(r, function(el) { return el.tagName == "fatalError"; });
						if (fatalError)
						{
							alert(fatalError.text || fatalError.textContent);
						}
						else //no fatal error
						{
							var guid = Utils.findFirstChild(r, function(el) { return el.tagName == "guid"; });

							if (survey && survey.parameters)
							{
								survey.parameters[9] = guid.firstChild.nodeValue;
							}

							var dcSiteUrl = Utils.findFirstChild(r, function(el) { return el.tagName == "url"; });
							var error = Utils.findFirstChild(r, function(el) { return el.tagName == "error"; });

							if (currentQuestion)
							{
								if (!currentQuestion.clickedLinks)
								{
									currentQuestion.clickedLinks = new Array();
								}
								currentQuestion.clickedLinks["_id" + link.id] = { "url": dcSiteUrl.firstChild.nodeValue, "hasError": (error ? true : false) };
							}

							if (error)
							{
								var errorDiv = document.createElement("div");
								errorDiv.className = "ErrorMsgBlock";

								for (var i = 0; i < error.childNodes.length; ++i)
								{
									if (error.childNodes[i].xml) //IE
									{
										errorDiv.innerHTML += error.childNodes[i].xml;
									}
									else //Firefox
									{
										errorDiv.appendChild(error.childNodes[i].cloneNode(true));
									}
								}

								// without this, Firefox will display errorDiv as literal but not HTML
								errorDiv.innerHTML = errorDiv.innerHTML;

								var errorBlockParent = link.parentNode.parentNode;
								var errorBlockNext = link.parentNode.nextSibling;

								if (currentQuestion)
								{
									errorBlockParent = currentQuestion.domObject;
									errorBlockNext = null;
									for (var i = 0; i < currentQuestion.domObject.childNodes.length; ++i)
									{
										if (Utils.isAncestorOf(currentQuestion.domObject.childNodes[i], link) &&
											currentQuestion.domObject.childNodes[i].type != "hidden")
										{
											errorBlockNext = currentQuestion.domObject.childNodes[i].nextSibling;
										}
									}
								}

								errorBlockParent.insertBefore(errorDiv, errorBlockNext);
							}
							else //no error
							{
								link.href = dcSiteUrl.firstChild.nodeValue;
								link.target = "_blank";
								cancelDefault = false;
							}
						}
					}
					else
					{
						alert(defaultErrorMsg);
					}
				}

				MS.Support.Fms.Gsfx.DataCollection.HandleDataCollectionLinkClick.isInProgress = false;
			}

			if (cancelDefault)
			{
				if (e.preventDefault)
				{
					e.preventDefault();
				}
				else
				{
					e.returnValue = false;
				}
			}
		}

		MS.Support.Fms.Gsfx.DataCollection.RequestManifestInstance = function()
		{
			var serviceHelperUrl = window.location.protocol + "//" + window.location.hostname + (window.location.port ? (":" + window.location.port) : "") + "/common/SDPWSWrapper.ashx";

			if (!window.XMLHttpRequest)
			{
				window.XMLHttpRequest = function()
				{
					return new ActiveXObject("Microsoft.XMLHTTP");
				}
			}

			try
			{
				var xmlHttpRequest = new XMLHttpRequest();

				xmlHttpRequest.open("GET", serviceHelperUrl, false);
				xmlHttpRequest.send("");
				return xmlHttpRequest.responseXML;
			}
			catch (e)
			{
				return null;
			}
		}
	}
}

if (typeof (kbSurvey) != "undefined")
{
	kbSurvey.onBeforeStart.add(new MS.Support.Fms.SurveyEventDelegate(null, MS.Support.Fms.Gsfx.GsfxSurvey.SurveyStartHandler));
	kbSurvey.start();
}
