if (!window.MS)
{
	MS = {};
}

if (!MS.Support)
{
	MS.Support = {};
}

if (!MS.Support.Fms)
{
	MS.Support.Fms = {};
}

if (!MS.Support.Fms)
{
	MS.Support.Fms = {};
}

if (!MS.Support.Fms.SurveyTrigger)
{
	MS.Support.Fms.SurveyTrigger = function(entity, config)
	{
		var cookieDomain = config ? config.site.cookieDomain : typeof (gCookieDomain) != "undefined" ? gCookieDomain : document.domain;

		var animationFPS = 24;
		var stduration = Math.round(1000 * (1 / animationFPS));

		var frameCount = entity.AnimationDuration * animationFPS;

		var movementPX = false;
		var surveyHeight;
		var surveyWidth;
		var surveyObj;
		var fixedPosition = false;
		var documentElement = null;

		function IsBackCompat()
		{
			if (document.compatMode == "BackCompat")
			{
				return true;
			}

			return false;
		}

		function checkFrequency()
		{
			return (!isNaN(entity.Frequency)) && (entity.Frequency > 0) && (Math.floor(Math.random() * entity.Frequency) == 0);
		}

		function isTriggerSuppressed(id)
		{
			var triggers = MS.Support.Fms.CookieUtil.getSubCookie('fmsmemo', 'st') || "";
			if ((triggers + '|').indexOf('|' + id + '|') > -1) return true;
			return false;
		}

		function suppressTriggger(id)
		{
			var triggers = MS.Support.Fms.CookieUtil.getSubCookie('fmsmemo', 'st') || "";
			triggers += '|' + id;
			MS.Support.Fms.CookieUtil.setSubCookie('fmsmemo', 'st', triggers, null, cookieDomain);
		}

		this.getSurveyId = function()
		{
			return parseInt(entity.Scid.split(";", 3)[2]) || 0;
		}

		this.getSurveyLanguageCode = function()
		{
			return entity.Scid.split(";", 3)[1] || "EN";
		}

		function getPreventMultipleResponsesCookieKey(surveyId, language)
		{
			return ("fmspmr_" + surveyId + "_" + language).toUpperCase();
		}

		// true indicate to be executed.
		// false indicate evaluation only.

		this.triggerInit = function(executing)
		{
			// Since the page will be cached by web server, so even we have suppressed a specific trigger,
			// it will still be contained in the page when we browse this page for the second time.
			// Thus we have to check if it has been suppressed at client side.
			if (isDomainTracking() || (entity.IntervalType == 'session' && isTriggerSuppressed(entity.TriggerId)))
			{
				return false;
			}

			var optOut = MS.Support.Fms.CookieUtil.getCookie("fmsOptOut" + entity.Site.toUpperCase());
			if (optOut && optOut == "1")
			{
				return false;
			}

			var surveyId = this.getSurveyId();

			if (surveyId != 0)
			{
				var key = getPreventMultipleResponsesCookieKey(this.getSurveyId(), this.getSurveyLanguageCode());

				if (MS.Support.Fms.CookieUtil.getCookie(key) == "1")
				{
					return false;
				}
			}

			var isFollowUp = entity.IntervalType == 'followup' ? true : false;

			if (isFollowUp)
			{
				var followupCookieKey = "fmsfollowups" + entity.CookieDef;
				if (!MS.Support.Fms.CookieUtil.getCookie(followupCookieKey))
				{
					// page is loaded from cache
					return false;
				}
				else if (executing)
				{
					// remove follow-up cookie
					MS.Support.Fms.CookieUtil.removeCookie(followupCookieKey, cookieDomain);
				}
			}

			var MiliDay = 86400000;
			var maturity = 0;
			var curDate = new Date();
			var visits = MS.Support.Fms.CookieUtil.getCookie(entity.CookieDef);

			var parts = null;

			if (visits)
			{
				parts = visits.split('_');
			}

			var expires = new Date();
			expires.setFullYear(expires.getFullYear() + 10);

			var ret;
			if ((!visits) || parts.length != 3 || isNaN(parts[0]))
			{
				MS.Support.Fms.CookieUtil.setCookie(entity.CookieDef, '1_0_0', expires, cookieDomain);
				parts = ["0", "0", "0"];
			}

			var origDate = parseInt(parts[1]);
			visits = parseInt(parts[0]);
			if
			(
				(executing || checkFrequency()) &&
				(visits >= maturity) && (isFollowUp || fmsSurveyExpired(entity.Expiration))
			)
			{
				if (executing)
					MS.Support.Fms.CookieUtil.setCookie(entity.CookieDef, visits + 1 + '_' + curDate.getTime() / MiliDay + '_' + entity.Expiration, expires, cookieDomain);
				ret = true;
			}
			else
			{
				if (executing)
					MS.Support.Fms.CookieUtil.setCookie(entity.CookieDef, visits + 1 + '_' + parts[1] + '_' + parts[2], expires, cookieDomain);
				ret = false;
			}

			if (!ret && entity.IntervalType == 'session')
				suppressTriggger(entity.TriggerId);
			return ret;
		}

		this.fireTrigger = function()
		{
			var querys = {
				"Scid": entity.Scid,
				"Xsl": entity.Xsl,
				"Site": entity.Site,
				"Tool": entity.Toolbar,
				"Theme": entity.Theme,
				"SD": entity.Site,
				"SurveyStyle": entity.SurveyStyle,
				"SiteRegion": entity.Region,
				"PageWidth": entity.PageWidth,
				"PageHeight": entity.PageHeight,
				"Url": entity.ReferringURL,
				"P0": entity.Parameters[0],
				"P1": entity.Parameters[1],
				"P2": entity.Parameters[2],
				"P3": entity.Parameters[3],
				"P4": entity.Parameters[4],
				"P5": entity.Parameters[5],
				"P6": entity.Parameters[6],
				"P7": entity.Parameters[7],
				"P8": entity.Parameters[8],
				"P9": entity.Parameters[9]
			};

			if (!window._ms_support_fms_utility_packageQueryString)
			{
				window._ms_support_fms_utility_packageQueryString = function(obj)
				{
					var queryString = "";
					for (var key in obj)
					{
						if (obj.hasOwnProperty(key))
						{
							queryString += "&" + key + "=" + encodeURIComponent(obj[key]);
						}
					}
					return queryString;
				}
			}

			var SurveyURL = entity.Path;

			if (config)
			{
				SurveyURL = config.protocol + "//" + config.host + SurveyURL;
			}

			var fullURL = SurveyURL + "?" + _ms_support_fms_utility_packageQueryString(querys);

			// parameters added in FMS 4.0
			if (typeof StatsDotNet != "undefined" && StatsDotNet)
			{
				// TODO: we need add these addtional parameters plugin trigger configuration snippet
				fullURL += '&ct=' + (StatsDotNet.contentType || "");
				fullURL += '&cc=' + (StatsDotNet.ContentCulture || "");
				fullURL += '&cid=' + (StatsDotNet.contentId || "");
				fullURL += '&clcid=' + (StatsDotNet.contentLn || "");
				fullURL += '&sc=' + (StatsDotNet.SiteCulture || "");
				fullURL += '&sbid=' + (StatsDotNet.siteBrandId || "");
				fullURL += '&ssid=' + (StatsDotNet.ssId || "");
				fullURL += '&ssver=' + (StatsDotNet.SsVersion || "");
				fullURL += '&cp=' + OutputEncoder_EncodeUrl(StatsDotNet.ContentProperties || "");
			}

			if (config)
			{
				querys = {
					'ct': config.content.type,
					'cc': config.content.culture,
					'cid': config.content.id,
					'clcid': config.content.lcid,
					'sc': config.site.culture,
					'sbid': config.site.brand,
					'ssid': config.site.id,
					'ssver': config.site.version,
					'cp': config.content.properties
				};

				fullURL += _ms_support_fms_utility_packageQueryString(querys);
			}

			fullURL += '&trigger=' + entity.TriggerId;

			if (entity.AltStyle)
			{
				fullURL += "&altStyle=" + entity.AltStyle + "&renderOption=" + entity.RenderOption;
			}

			if (entity.EmailStyle == 1)
				fullURL = fullURL + '&emailsurveyid=' + entity.EmailSurveyID + '&sessionid=-1';

			if (entity.DisplayIntroPage != '1')
				fullURL = fullURL + '&showpage=1';
			if (entity.SurveyStyle == null)
				entity.SurveyStyle = "popup";
			entity.SurveyStyle = entity.SurveyStyle.toLowerCase();
			if (entity.SurveyStyle == "embedded")
			{
				if (entity.EmbedSurveyPrompt == 'nothing')
					window.location.href = fullURL;
				else
				{
					if (entity.EmbedSurveyPrompt != "" && window.screenTop < 10000 && window.confirm(embedSurveyPrompt) == true) //unload event
					{
						// we can not do a window.location.href for the redirection because when refreshing the window that redirection won't work
						// Optimally if it is refreshing, we should not do survey. But we have no way to know it is refreshing window, so we keep the behavior same
						document.writeln('<html><body>');
						document.writeln('<form name="the_form" action="' + fullURL + '" method="post"><\/form>');
						document.writeln("<\/body><\/html>");
						document.the_form.submit();
					}
				}
			}
			else
			{
				fullURL = fullURL.toLowerCase().replace("survey.aspx", "surveyinvite.aspx");
				if (entity.DisplayIntroPage == '1')
					fullURL = fullURL + '&showpage=1'; //always add &showpage=1
				if (document.location.href.toLowerCase().indexOf('fr=1') > 0)
					fullURL = fullURL + '&fr=1';
				if (entity.Event == 'onunload')
					fullURL = fullURL + '&onunload=1';
				if (entity.OptOut == 1)
				{
					fullURL = fullURL + "&optout=1";
				}
				fireSurvey(fullURL);
			}
		}

		function fireSurvey(fullURL)
		{
			presentSurvey(fullURL);
		}

		function surveyDiv(fullURL)
		{
			var sd = document.createElement("div");
			sd.id = "surveyDivBlock";
			sd.className = "surveyInvitationDiv";
			var si = document.createElement("iframe");
			si.scrolling = 'no';
			si.frameBorder = 0;
			si.name = "fmsInvitation";
			si.id = "fmsInvitation";

			if (!config)
			{
				if (entity.InvitationWidth)
				{
					var width = entity.InvitationWidth + "px";
					si.style.width = width;
					sd.style.width = width;
				}
				si.src = fullURL;
			}
			else
			{
				var width = config.invitation.width || ((entity.InvitationWidth || 600) + "px");
				si.style.width = width;
				sd.style.width = width;

				function buildPluginTriggerInvitation()
				{
					// verify explicit domain again, because document.domain may has been changed after the trigger snippet
					var explicitDomain = (window.location.hostname != document.domain) ? true : config.site.explicitDomain;

					var parts = entity.Scid.split(";");

					config.entity = entity;

					var surveyConfig = {
						"target": "SurveyInvitationContent",
						"template": "default",
						"enableLTS": false,
						"survey": {
							"host": config.host,
							"language": parts[1],
							"id": parts[2],
							"isRTL": entity.IsRTL,
							"features": ["Title,Introduction," + (entity.OptOut ? "OptOut," : "") + "AcceptButton,DeclineButton", ""],
							"isInvitation": 1
						},
						"parameters": config.parameters,
						"site": {
							"name": config.site.name,
							"culture": config.site.culture,
							"lcid": config.site.lcid,
							"id": parseInt(config.site.id) || 0,
							"brand": parseInt(config.site.brand) || 0,
							"version": config.site.version || "",
							"explicitDomain": explicitDomain,
							"cookieDomain": config.site.cookieDomain
						},
						"content": {
							"type": config.content.type,
							"id": config.content.id,
							"culture": config.content.culture,
							"lcid": config.content.lcid,
							"properties": config.content.properties,
							"aggregateId": config.content.aggregateId
						},
						"triggerConfig": config
					};

					var style = "<style type=\"text/css\">body{direction:" + (entity.IsRTL ? "rtl" : "ltr") + "}</style>";
					var header = "<div id=\"header\">" + config.invitation.header + "</div>";
					var footer = "<div id=\"footer\">" + config.invitation.footer + "</div>";
					var script = explicitDomain ? ("<script type=\"text/javascript\">document.domain=\"" + document.domain + "\";</scri" + "pt>") : "";

					var iframeHtml = "<!DOCTYPE html PUBLIC \"-//W3C//DTD XHTML 1.0 Transitional//EN\" \"http://www.w3.org/TR/xhtml1/DTD/xhtml1-transitional.dtd\">" +
											"<html>" +
												"<head>" +
													style +
													script +
												"</head>" +
												"<body style=\"margin:0\">" +
													"<div id=\"wrapper\">" +
														header +
														"<div id=\"content\">" +
															"<div id=\"SurveyInvitationContent\"></div>" +
														"</div>" +
														footer +
													"</div>" +
												"</body>" +
												"<script type=\"text/javascript\" src =" + config.protocol + "//" + config.host + "/common/script/fx/surveystrapper.js></scr" + "ipt>" +
											"</html>";

					var src = "about:blank";

					if (explicitDomain)
					{
						if (window.navigator.userAgent.indexOf("MSIE") > -1 || (window.opera && parseFloat(opera.version()) >= 9.5))
						{
							MS.Support.Fms.SurveyTrigger.surveyConfig = surveyConfig;
							MS.Support.Fms.SurveyTrigger.iframeHtml = iframeHtml;
							src = "javascript:(function(){document.open();document.domain=\"" + document.domain + "\";window[\"_ms_support_fms_surveyConfig\"]=window.parent.MS.Support.Fms.SurveyTrigger.surveyConfig;document.write(window.parent.MS.Support.Fms.SurveyTrigger.iframeHtml);document.close();})();";
						}
						else if (window.opera)
						{
							src = "javascript:(function(){document.open();document.domain=\"" + document.domain + "\";document.close();})()";
						}
					}

					si.src = src;
					config.fullSurveyUrl = fullURL;

					function setupInvitationFrame()
					{
						try
						{
							var frameWindow = window["fmsInvitation"];
							var frameDocument = frameWindow.document;
							frameDocument.open();
							frameWindow["_ms_support_fms_surveyConfig"] = surveyConfig;
							frameDocument.write(iframeHtml);
							frameDocument.close();
						}
						catch (ex)
						{
							// if document.domain was changed after trigger snippet, access the document may cause an exception
						}
					}

					if ((!explicitDomain) || (window.navigator.userAgent.indexOf("MSIE") == -1))
					{
						setupInvitationFrame();
					}
				}

				// setTimeout is required here to address a bug (75855) on Firefox 3.0 (not repro on Firefox 3.5)
				window.setTimeout(buildPluginTriggerInvitation, 0);
			}

			sd.appendChild(si);
			documentElement.appendChild(sd);

			return sd;
		}

		function hI(name, value)
		{
			var inp = document.createElement("input");
			inp.type = "hidden";
			inp.name = name;
			inp.value = value;
			return inp;
		}

		function declineSurvey(el)
		{
			while (el.className != "surveyInvitationDiv") el = el.parentNode;
			el.parentNode.removeChild(el);
		}

		function closeEnough(int1, int2)
		{
			if (Math.abs(int1 - int2) <= movementPX) return true;
			return false;
		}

		function calcXY(current, target)
		{
			if (!closeEnough(current, target))
			{
				var delta = current - target;
				var dir = delta / Math.abs(delta);
				return current - movementPX * dir;
			}

			return false;
		}

		function isRTL()
		{
			if ((!fixedPosition) && documentElement.currentStyle && documentElement.currentStyle.blockDirection == "rtl")
			{
				return true;
			}
			else
			{
				return false;
			}
		}

		function getFrameX()
		{
			var x = parseInt(isRTL() ? surveyObj.style.right : surveyObj.style.left);
			return isNaN(x) ? 0 : x;
		}

		function getFrameY()
		{
			var y = parseInt(surveyObj.style.top);
			return isNaN(y) ? 0 : y;
		}

		function getTargetFrameX()
		{
			return getWindowCenterX() - surveyWidth / 2;
		}

		function getTargetFrameY()
		{
			return getWindowCenterY() - surveyHeight / 2;
		}

		function setFrameX(x)
		{
			if (isRTL())
			{
				surveyObj.style.right = x + "px";
			}
			else
			{
				surveyObj.style.left = x + "px";
			}
		}

		function setFrameY(y)
		{
			surveyObj.style.top = y + "px";
		}

		function moveFrameTo(x, y)
		{
			setFrameX(x);
			setFrameY(y);
		}

		function getScrollX()
		{
			if (!fixedPosition)
			{
				if (isRTL())
				{
					return (standardDocument.scrollWidth - standardDocument.clientWidth - standardDocument.scrollLeft);
				}
				else
				{
					return standardDocument.scrollLeft;
				}
			}
			else
			{
				return 0;
			}
		}

		function getScrollY()
		{
			return (!fixedPosition) ? standardDocument.scrollTop : 0;
		}

		function getWindowCenterX()
		{
			return Math.round(standardDocument.clientWidth / 2 + getScrollX());
		}

		function getWindowCenterY()
		{
			return Math.round(standardDocument.clientHeight / 2 + getScrollY());
		}

		function animateSurvey()
		{
			calcFPS();
			var x = calcXY(getFrameX(), getTargetFrameX());
			var y = calcXY(getFrameY(), getTargetFrameY());
			if (x != false || y != false)
			{
				if (x != false)
				{
					setFrameX(x);
				}

				if (y != false)
				{
					setFrameY(y);
				}

				setTimeout(animateSurvey, stduration);
			}
		}

		function getPositionDeltaX()
		{
			return Math.abs(getTargetFrameX() - getFrameX());
		}

		function getPositionDeltaY()
		{
			return Math.abs(getTargetFrameY() - getFrameY());
		}

		function fbp(p1, p2)
		{
			if (!p1 || p1 < 1)
			{
				return p2;
			}
			return p1;
		}

		function calcFPS()
		{
			var x = getPositionDeltaX();
			var y = getPositionDeltaY();

			if (y > x) x = y;

			movementPX = Math.ceil(x / frameCount);
		}

		window.placeSurvey = function()
		{
			surveyWidth = fbp(surveyObj.clientWidth, surveyObj.offsetWidth);
			surveyHeight = fbp(surveyObj.clientHeight, surveyObj.offsetHeight);
			setFrameX(getTargetFrameX());
			surveyObj.style.top = 0;
			surveyObj.style.visibility = "visible";

			if (window.addEventListener)
			{
				window.addEventListener("resize", animateSurvey, false);
				surveyObj.style.position = "fixed";
				fixedPosition = true;
			}
			else if (window.attachEvent)
			{
				window.attachEvent("onresize", animateSurvey);
				window.attachEvent("onscroll", animateSurvey);
			}

			animateSurvey();
		}

		function presentSurvey(fullURL)
		{
			documentElement = document.body ? document.body : document.documentElement;
			// this object is for getting clientWidth, clientHeight etc in different DOCTYPEs.
			standardDocument = IsBackCompat() ? documentElement : document.documentElement;
			if (documentElement)
			{
				surveyObj = surveyDiv(fullURL);
			}
		}


		if (this.triggerInit(false))
		{
			var trigger = this;
			setTimeout(
					function ()
					{
						if (trigger.triggerInit(true))
						{
							trigger.fireTrigger();
						}
					},
					entity.Delay
				);
		}
		else if (typeof (activateSiteSurvey) != "undefined")
		{
			activateSiteSurvey();
		}

	}
}
