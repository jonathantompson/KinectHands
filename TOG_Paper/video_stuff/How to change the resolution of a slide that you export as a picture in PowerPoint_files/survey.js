if (!window.MS)
{
	window.MS = {};
}

if (!MS.Support)
{
	MS.Support = {};
}

if (!MS.Support.Fms)
{
	MS.Support.Fms = {};
}

if (!MS.Support.Fms.Survey)
{
	MS.Support.Fms.Utils =
	{
		findChildNodes: function (parent, evaluator)
		{
			var elements = [];

			if (evaluator(parent))
			{
				elements.push(parent);
			}

			if (parent != null && parent.childNodes)
			{
				for (var i = 0; i < parent.childNodes.length; ++i)
				{
					var es = arguments.callee(parent.childNodes[i], evaluator);
					for (var j = 0; j < es.length; ++j)
					{
						elements.push(es[j]);
					}
				}
			}

			return elements;
		},

		findFirstChild: function (parent, evaluator)
		{
			if (evaluator(parent))
			{
				return parent;
			}
			else if (parent != null && parent.childNodes)
			{
				for (var index = 0; index < parent.childNodes.length; ++index)
				{
					var e = arguments.callee(parent.childNodes[index], evaluator);

					if (e != null)
					{
						return e;
					}
				}
			}

			return null;
		},

		getChildById: function (parent, childId)
		{
			if (parent == null)
			{
				return null;
			}
			else if (parent.getElementById)
			{
				return parent.getElementById(childId);
			}
			else if (parent.querySelector)
			{
				return parent.querySelector("#" + childId);
			}
			else
			{
				return MS.Support.Fms.Utils.findFirstChild(
					parent,
					function (parent)
					{
						return (parent != null && parent.id == childId);
					}
				);
			}

		},

		getChildByAttribute: function (parent, attributeName, attributeValue)
		{
			if (parent.querySelector)
			{
				return parent.querySelector("[" + attributeName + "=" + attributeValue + "]");
			}
			else
			{
				return MS.Support.Fms.Utils.findFirstChild(
					parent,
					function (parent)
					{
						return parent != null && parent.getAttribute ? (parent.getAttribute(attributeName) == attributeValue) : false;
					}
				);
			}
		},

		getChildNodesByAttribute: function (parent, attributeName, attributeValue)
		{
			if (parent.querySelectorAll)
			{
				return parent.querySelectorAll("[" + attributeName + "=" + attributeValue + "]");
			}
			else
			{
				return MS.Support.Fms.Utils.findChildNodes(
					parent,
					function (parent)
					{
						return parent != null && parent.getAttribute ? (parent.getAttribute(attributeName) == attributeValue) : false;
					}
				);
			}
		},

		getChildByName: function (parent, name)
		{
			return MS.Support.Fms.Utils.getChildByAttribute(parent, "name", name);
		},

		getChildByTagName: function (parent, tagName)
		{
			if (parent.querySelector)
			{
				return parent.querySelector(tagName);
			}
			else
			{
				return MS.Support.Fms.Utils.findFirstChild(
					parent,
					function (e)
					{
						return e && e.tagName && e.tagName.toUpperCase() == tagName.toUpperCase();
					}
				);
			}
		},

		getChildNodesByClass: function (parent, className)
		{
			if (parent.querySelectorAll)
			{
				return parent.querySelectorAll("." + className);
			}
			else
			{
				return MS.Support.Fms.Utils.findChildNodes(
					parent,
					function (e)
					{
						return MS.Support.Fms.Utils.hasClass(e, className);
					}
				);
			}
		},

		isAncestorOf: function (ancestor, posterity)
		{
			if (ancestor.contains)
			{
				return ancestor.contains(posterity);
			}

			var parent = posterity;
			while (parent != null)
			{
				if (ancestor == parent)
				{
					return true;
				}
				parent = parent.parentNode;
			}

			return false;
		},

		randomize: function (array)
		{
			var len = array.length;
			for (var i = 0; i < len; ++i)
			{
				var x = Math.floor(Math.random() * (len - i)) + i;

				if (x != i)
				{
					var t = array[i];
					array[i] = array[x];
					array[x] = t;
				}
			}
		},

		swapNode: function (node1, node2)
		{
			if (node1.swapNode)
			{
				node1.swapNode(node2);
			}
			else
			{
				var parent1 = node1.parentNode;
				var parent2 = node2.parentNode;

				var s2 = node2.nextSibling;

				parent1.insertBefore(node2, node1);

				if (s2)
				{
					parent2.insertBefore(node1, s2);
				}
				else
				{
					parent2.appendChild(node1);
				}
			}
		},

		addEventHandler: function (e, eventName, fp)
		{
			if (e.addEventListener)
			{
				e.addEventListener(eventName, fp, false);
			}
			else if (e.attachEvent)
			{
				e.attachEvent("on" + eventName, fp);
			}
		},

		removeEventHandler: function (e, eventName, fp)
		{
			if (e.removeEventListener)
			{
				e.removeEventListener(eventName, fp, false);
			}
			else if (e.detachEvent)
			{
				e.detachEvent("on" + eventName, fp);
			}
		},

		getCookie: function (key)
		{
			var entities = document.cookie.split(";");
			for (var i = 0; i < entities.length; ++i)
			{
				var j = entities[i].indexOf("=");
				var s = entities[i].substring(0, j);
				if (s != "" && (s == key || s == " " + key))
				{
					return entities[i].substring(j + 1);
				}
			}
			return null;
		},

		setCookie: function (key, value, expires, domain, path)
		{
			domain = domain || (typeof (gCookieDomain) != "undefined" ? (gCookieDomain || document.domain) : document.domain);
			path = path || "/";
			document.cookie = key + "=" + value + "; domain=" + domain + "; path=" + path + "; " + (expires ? ("expires=" + expires.toGMTString() + ";") : "");
		},

		setSessionCookie: function (key, value, domain, path)
		{
			this.setCookie(key, value, null, domain, path);
		},

		htmlEncode: function (sourceString)
		{
			var encodedString = "";
			if (sourceString.length == 0)
			{
				return "";
			}

			for (var i = 0; i < sourceString.length; i++)
			{
				switch (sourceString.substr(i, 1))
				{
					case "<":
						encodedString += "&lt;";
						break;
					case ">":
						encodedString += "&gt;";
						break;
					case "&":
						encodedString += "&amp;";
						break;
					case "\"":
						encodedString += "&quot;";
						break;
					case " ":
						encodedString += "&nbsp;";
						break;
					case "\n":
						encodedString += "<br />";
						break;
					default:
						encodedString += sourceString.substr(i, 1);
						break;
				}
			}

			return encodedString;
		},

		characterEncode: function (sourceString)
		{
			var encodedString = "";
			if (sourceString.length == 0)
			{
				return "";
			}

			for (var i = 0; i < sourceString.length; i++)
			{
				switch (sourceString.substr(i, 1))
				{
					case "$":
						encodedString += "$$";
						break;
					default:
						encodedString += sourceString.substr(i, 1);
						break;
				}
			}

			return encodedString;
		},

		hideElement: function (e)
		{
			e.style.display = "none";
		},

		displayTableRow: function (row)
		{
			try
			{
				row.style.display = "table-row";
			}
			catch (ex)
			{
				row.style.display = "block";
			}
		},

		displayTableCell: function (cell)
		{
			try
			{
				cell.style.display = "table-cell";
			}
			catch (ex)
			{
				cell.style.display = "block";
			}
		},

		getUrlParameters: function ()
		{
			var parameters = window.location.search.substring(1).split('&');
			var queryFields = {};
			for (var i = 0; i < parameters.length; i++)
			{
				var param = parameters[i];
				var index = param.indexOf('=');
				var key = decodeURIComponent(index != -1 ? param.substring(0, index).toLowerCase() : param);
				var value = index != -1 ? decodeURIComponent(param.substring(index + 1, param.length)) : "";
				if (queryFields[key])
				{
					queryFields[key] += "," + value;
				}
				else
				{
					queryFields[key] = value;
				}
			}

			return queryFields;
		},

		hasClass: function (e, className)
		{
			if (!e || typeof (e.className) != "string" || !className || typeof (className) != "string")
			{
				return false;
			}

			var classes = " " + e.className.toUpperCase() + " ";
			return classes.indexOf(" " + className.toUpperCase() + " ") != -1;
		},

		addClass: function (e, className)
		{
			if (!e || typeof (e.className) != "string" || !className || typeof (className) != "string" || MS.Support.Fms.Utils.hasClass(e, className))
			{
				return;
		}

			e.className += " " + className;
		},

		removeClass: function (e, classNames)
		{
			if (!e || typeof (e.className) != "string" || !classNames || typeof (classNames) != "string")
			{
				return;
			}

			var toBeRemoved = classNames.split(/\s+/);
			var className = (" " + e.className + " ").replace(/[\n\t\r]/g, " ");
			var c, cl;
			for (c = 0, cl = toBeRemoved.length; c < cl; c++)
			{
				className = className.replace(" " + toBeRemoved[c] + " ", " ");
			}

			e.className = className.replace(/(^\s*)|(\s*$)/g, "");
		},

		getToday: function ()
		{
			var currentDate = new Date();
			return (currentDate.getFullYear() + "-" + ("0" + (currentDate.getMonth() + 1)).slice(-2) + "-" + ("0" + currentDate.getDate()).slice(-2));
		}

	};

	MS.Support.Fms.ValidateResult = function (isValid, errorMessage)
	{
		this.isValid = isValid;
		this.errorMessage = errorMessage;
	}

	MS.Support.Fms.SurveyEventDelegate = function (object, method)
	{
		this.object = object;
		this.method = method;

		this.invoke = function ()
		{
			if (method != null)
			{
				return method.apply(object, arguments);
			}
		}

		this.equal = function (another)
		{
			return this.object == another.object && this.method == another.method;
		}
	}

	MS.Support.Fms.SurveyEvent = function ()
	{
		var handlers = [];

		this.add = function (delegate)
		{
			for (var index = 0; index < handlers.length; ++index)
			{
				if (handlers[index].equal(delegate))
				{
					return;
				}
			}

			handlers.push(delegate);
		}

		this.remove = function (delegate)
		{
			for (var index = 0; index < handlers.length; ++index)
			{
				if (handlers[index].equal(delegate))
				{
					handlers.splice(index, 1);
					return;
				}
			}
		}

		this.fire = function ()
		{
			for (var index = 0; index < handlers.length; ++index)
			{
				if (!handlers[index].invoke.apply(null, arguments))
				{
					return false;
				}
			}

			return true;
		}
	}

	MS.Support.Fms.SurveyQuestionOperator =
	{
		// creare a operator for a given question type
		create: function (question)
		{
			var operatorPrototype = null;

			switch (question.getQuestionType())
			{
				case "CHOICE":
				case "CHECK-LIST":
					operatorPrototype = this.choiceOperator;
					break;
				case "CHOICE-SEQ":
				case "CHOICE-HOR":
					operatorPrototype = this.ghrOperator;
					break;
				case "CHOICE-LIST":
				case "MULTI-OPTION":
					operatorPrototype = this.choicelistOperator;
					break;

				case "TEXT-LINE":
				case "PASSWORD":
				case "TEXT-BLOCK":
					operatorPrototype = this.textboxOperator;
					break;
				case "TEXT-DATE":
					operatorPrototype = this.dateOperator;
					break;
				case "TEXT-NUMERIC":
					operatorPrototype = this.numericOperator;
					break;

				case "LABEL":
				default:
					operatorPrototype = this.emptyOperator;
					break;
			}

			return new operatorPrototype(question);
		},

		emptyOperator: function (question)
		{
			this.validateRequired = function ()
			{
				return true;
			}

			this.validateInput = function ()
			{
				return new MS.Support.Fms.ValidateResult(true);
			}

			this.prepare = function ()
			{
			}

			this.populateVariables = function ()
			{
				return [];
			}

			this.randomize = function ()
			{
			}

			this.getAnswers = function ()
			{
				return [];
			}

			this.save = function ()
			{
			}

			this.restore = function ()
			{
			}

			this.getOptionById = function ()
			{
				return null;
			}
		},

		textboxOperator: function (question)
		{
			var textbox = question.elements[0];

			this.validateRequired = function ()
			{
				return textbox.value != "";
			}

			this.validateInput = function ()
			{
				return new MS.Support.Fms.ValidateResult(true);
			}

			this.prepare = function ()
			{
			}

			this.populateVariables = function ()
			{
				var variables = [];

				var variable = question.domObject.getAttribute("var");

				if (variable != null && variable != "")
				{
					if (textbox.value != null && textbox.value != "")
					{
						variables[variable] = textbox.value;
					}
					else
					{
						variables[variable] = null;
					}
				}

				return variables;
			}

			this.getOptionById = function ()
			{
				return null;
			}

			this.randomize = function ()
			{
			}

			this.getAnswers = function ()
			{
				var answers = [];
				var text = textbox.value;
				if (text != null && text != "")
				{
					answers.push({ id: 0, value: 0, text: text });
				}
				return answers;
			}

			var savedText;

			this.save = function ()
			{
				savedText = textbox.value;
			}

			this.restore = function ()
			{
				textbox.value = savedText;
			}
		},

		dateOperator: function (question)
		{
			var Fms = MS.Support.Fms;
			var Utils = Fms.Utils;
			var ValidateResult = Fms.ValidateResult;

			var baseOperator = new Fms.SurveyQuestionOperator.textboxOperator(question);
			var textbox = question.elements[0];
			var dateErrorText = textbox.getAttribute("data-val-dateerrortext");
			this.validateRequired = function ()
			{
				return baseOperator.validateRequired();
			}
			this.validateInput = function ()
			{
				var valueString = textbox.value;
				if (valueString != "")
				{
					var re = /^(?:(?!0000)[0-9]{4}([-])(?:(?:0?[1-9]|1[0-2])\1(?:0?[1-9]|1[0-9]|2[0-8])|(?:0?[13-9]|1[0-2])\1(?:29|30)|(?:0?[13578]|1[02])\1(?:31))|(?:[0-9]{2}(?:0[48]|[2468][048]|[13579][26])|(?:0[48]|[2468][048]|[13579][26])00)([-])0?2\2(?:29))$/i;
					if (!(re.test(valueString)))
					{
						return new ValidateResult(false, dateErrorText);
					}
				}
				return new ValidateResult(true);
			}

			this.prepare = function ()
			{
				return baseOperator.prepare();
			}

			this.populateVariables = function ()
			{
				return baseOperator.populateVariables();
			}

			this.getOptionById = function ()
			{
				return baseOperator.getOptionById();
			}

			this.randomize = function ()
			{
				return baseOperator.randomize();
			}

			this.getAnswers = function ()
			{
				return baseOperator.getAnswers();
			}

			this.save = function ()
			{
				return baseOperator.save();
			}

			this.restore = function ()
			{
				return baseOperator.restore();
			}

		},

		numericOperator: function (question)
		{
			var Fms = MS.Support.Fms;
			var Utils = Fms.Utils;
			var ValidateResult = Fms.ValidateResult;

			var baseOperator = new Fms.SurveyQuestionOperator.textboxOperator(question);

			var textbox = question.elements[0];
			var enableFloat = Utils.getChildByName(question.domObject, "enableFloat").value == "1" ? true : false;
			var minValue = textbox.getAttribute("min");
			var maxValue = textbox.getAttribute("max");
			var numericErrorText = Utils.getChildByName(question.domObject, "numericErrorText").value;

			this.validateRequired = function ()
			{
				return baseOperator.validateRequired();
			}

			this.validateInput = function ()
			{
				var valueString = textbox.value;
				if (valueString != "")
				{
					var parse = enableFloat ? parseFloat : parseInt;

					var value = parse(valueString);

					var re = enableFloat ? /^(\+|-)?((\d+(\.\d*)?)|(\.\d+))(e(\+|-)?\d+)?$/i : /^(\+|-)?\d+$/i;

					if (isNaN(value)
						|| (!re.test(valueString))
						|| (minValue != "" && value < parse(minValue))
						|| (maxValue != "" && value > parse(maxValue))
					)
					{
						return new ValidateResult(false, numericErrorText);
					}
				}

				return new ValidateResult(true);
			}

			this.prepare = function ()
			{
				return baseOperator.prepare();
			}

			this.populateVariables = function ()
			{
				return baseOperator.populateVariables();
			}

			this.getOptionById = function ()
			{
				return baseOperator.getOptionById();
			}

			this.randomize = function ()
			{
				return baseOperator.randomize();
			}

			this.getAnswers = function ()
			{
				return baseOperator.getAnswers();
			}

			this.save = function ()
			{
				return baseOperator.save();
			}

			this.restore = function ()
			{
				return baseOperator.restore();
			}
		},

		choiceOperator: function (question)
		{
			var Utils = MS.Support.Fms.Utils;

			var lastRadioIndex = question.elements.length - 1;
			var optionalTextBox = null;

			var hasOptionalTextBox = lastRadioIndex >= 0 ? question.elements[lastRadioIndex].type.toLowerCase() == "text" : false;

			if (hasOptionalTextBox)
			{
				optionalTextBox = question.elements[lastRadioIndex];
				--lastRadioIndex;
			}

			var rows = null;

			function findRows()
			{
				var table = Utils.getChildByTagName(question.domObject, "TABLE");

				if (table != null)
				{
					rows = table.rows;
				}
			}

			findRows();

			function handleOptionalTextBox(e)
			{
				if (hasOptionalTextBox)
				{
					optionalTextBox.disabled = !question.elements[lastRadioIndex].checked;
				}
			}

			function isExclusiveOption(e)
			{
				return e && e.type == "checkbox" && e.getAttribute && e.getAttribute("exclusive") == "1";
			}

			function getElementContainer(e)
			{
				return e.parentNode.parentNode.parentNode.parentNode;
			}

			function handleExclusiveOption(event)
			{
				var evt = event || window.event;
				var element = evt.srcElement || evt.target;

				if (isExclusiveOption(element))
				{
					if (element.checked)
					{
						for (var index = 0; index <= lastRadioIndex; ++index)
						{
							var e = question.elements[index];
							if (e != element)
							{
								e.checked = false;
								e.disabled = true;
								getElementContainer(e).disabled = true;
							}
						}
					}
					else
					{
						for (var index = 0; index <= lastRadioIndex; ++index)
						{
							var e = question.elements[index];
							if (e.disabled == true)
							{
								e.disabled = false;
								getElementContainer(e).disabled = false;
							}
						}
					}
				}
			}

			function handleOptionClick(e)
			{
				handleExclusiveOption(e);
				handleOptionalTextBox(e);
			}

			for (var index = 0; index <= lastRadioIndex; ++index)
			{
				var e = question.elements[index];
				Utils.addEventHandler(e, "click", handleOptionClick);
			}

			this.validateRequired = function ()
			{
				if (lastRadioIndex == 0)
				{
					return true;
				}
				for (var index = 0; index <= lastRadioIndex; ++index)
				{
					var e = question.elements[index];
					if (e.visible && e.checked)
					{
						if (hasOptionalTextBox && question.elements[lastRadioIndex].checked)
						{
							return (optionalTextBox.value != "");
						}

						return true;
					}
				}

				return false;
			}

			this.validateInput = function ()
			{
				return new MS.Support.Fms.ValidateResult(true);
			}

			this.prepare = function ()
			{
				if (rows.length <= 0)
				{
					return;
				}

				var columns = rows[0].cells.length;

				if (columns <= 0)
				{
					return;
				}

				var rowIndex = 0, cellIndex = 0;
				for (var index = 0; index <= lastRadioIndex; ++index)
				{
					var e = question.elements[index];
					var bShow = true, bSelect = e.checked;
					var visibility = e.getAttribute("visibility");

					var retVal = question.survey.EvaluateContextVisibility(e);

					switch (visibility)
					{
						case "Show":
							bShow = retVal == "NotExist" ? false : retVal == "EvalTrue" ? true : false;
							break;
						case "Hide":
							bShow = retVal == "NotExist" ? true : retVal == "EvalTrue" ? false : true;
							break;
						case "Selected":
							bShow = true;
							bSelect = retVal == "NotExist" ? false : retVal == "EvalTrue" ? true : false;
							break;
						default:
							break;
					}

					if (bShow)
					{
						e.visible = true;
						if (bSelect)
						{
							if (!e.checked)
							{
								e.click();
							}
						}
						else
						{
							if (e.checked)
							{
								e.checked = false;
							}
						}

						var row = rows[rowIndex];
						var cell = row.cells[cellIndex];

						Utils.displayTableRow(row);
						Utils.displayTableCell(cell);

						var container = getElementContainer(e);

						if (cell.firstChild != null)
						{
							if (cell.firstChild != container)
							{
								Utils.swapNode(cell.firstChild, container);
							}
						}
						else
						{
							cell.appendChild(container);
						}

						if (index == lastRadioIndex && hasOptionalTextBox)
						{
							cell.appendChild(getElementContainer(optionalTextBox));
						}

						++cellIndex;
						if (cellIndex == columns)
						{
							cellIndex = 0;
							++rowIndex;
						}
					}
					else
					{
						e.visible = false;
						if (index == lastRadioIndex && hasOptionalTextBox)
						{
							rows[rows.length - 1].cells[columns - 1].appendChild(getElementContainer(optionalTextBox));
						}
					}
				}

				var isQuestionNeedHide = true;
				for (var index = 0; index <= lastRadioIndex; ++index)
				{
					var e = question.elements[index];
					if (e.visible)
					{
						isQuestionNeedHide = false;
					}
				}
				if (isQuestionNeedHide)
				{
					question.needHide = true;
				}

				if (cellIndex != 0 && rowIndex < rows.length)
				{
					for (; cellIndex < columns; ++cellIndex)
					{
						Utils.hideElement(rows[rowIndex].cells[cellIndex]);
					}
					++rowIndex;
				}

				for (; rowIndex < rows.length; ++rowIndex)
				{
					Utils.hideElement(rows[rowIndex]);
				}
			}

			function isFixed(element)
			{
				return (element.getAttribute("fixed") == "1");
			}

			this.randomize = function ()
			{
				if (question.needRandomization() && lastRadioIndex > 0)
				{
					var randomizedOptions = [];
					var nonFixedOptions = [];

					for (var index = 0; index < lastRadioIndex; ++index)
					{
						if (isFixed(question.elements[index]))
						{
							randomizedOptions[index] = question.elements[index];
						}
						else
						{
							nonFixedOptions.push(index);
						}
					}

					if (hasOptionalTextBox || isFixed(question.elements[lastRadioIndex]))
					{
						randomizedOptions[lastRadioIndex] = question.elements[lastRadioIndex];
					}
					else
					{
						nonFixedOptions.push(lastRadioIndex);
					}

					Utils.randomize(nonFixedOptions);

					for (var index = 0; index <= lastRadioIndex; ++index)
					{
						if (randomizedOptions[index] == null)
						{
							randomizedOptions[index] = question.elements[nonFixedOptions.shift()];
						}
					}

					for (var i = 0; i <= lastRadioIndex; ++i)
					{
						if (randomizedOptions[i] != question.elements[i])
						{
							for (var j = 0; j <= lastRadioIndex; ++j)
							{
								if (question.elements[j] == randomizedOptions[i])
								{
									question.elements[j] = question.elements[i];
									question.elements[i] = randomizedOptions[i];

									break;
								}
							}
						}
					}
				}
			}

			this.getAnswers = function ()
			{
				var answers = [];

				for (var index = 0; index <= lastRadioIndex; ++index)
				{
					var e = question.elements[index];
					if (e.visible && e.checked)
					{
						var id = e.id, value = e.value, text = "";

						if (index == lastRadioIndex && hasOptionalTextBox)
						{
							text = optionalTextBox.value;
						}

						answers.push({ id: id, value: value, text: text });
					}
				}

				return answers;
			}

			var states = null;

			this.save = function ()
			{
				states = [];

				for (var index = 0; index <= lastRadioIndex; ++index)
				{
					states.push(question.elements[index].checked);
				}

				if (hasOptionalTextBox)
				{
					states.push({ value: optionalTextBox.value, disabled: optionalTextBox.disabled });
				}
			}

			function getOptionText(index)
			{
				var e = question.elements[index];
				return (e.visible && e.checked) ? ((index == lastRadioIndex && hasOptionalTextBox) ? optionalTextBox.value : e.parentNode.nextSibling.innerHTML) : null;
			}

			function getOptionVariable(index)
			{
				var e = question.elements[index];
				var variable = e.getAttribute("var");

				if (variable != null && variable != "")
				{
					return { name: variable, value: getOptionText(index) };
				}

				return null;
			}

			function getQuestionVariable()
			{
				var variable = question.domObject.getAttribute("var");

				if (variable != null && variable != "")
				{
					var value = "";
					for (var index = 0; index <= lastRadioIndex; ++index)
					{
						var e = question.elements[index];
						if (e.visible && e.checked)
						{
							value += ", " + getOptionText(index);
						}
					}

					return { name: variable, value: (value.length > 2 ? value.substring(2) : null) };
				}

				return null;
			}

			this.populateVariables = function ()
			{
				var variables = [];

				var questionVariable = getQuestionVariable();

				if (questionVariable != null)
				{
					variables[questionVariable.name] = questionVariable.value;
				}

				for (var index = 0; index <= lastRadioIndex; ++index)
				{
					var optionVariable = getOptionVariable(index);

					if (optionVariable != null)
					{
						variables[optionVariable.name] = optionVariable.value;
					}
				}

				return variables;
			}

			this.getOptionById = function (optionId)
			{
				for (var index = 0; index <= lastRadioIndex; ++index)
				{
					if (question.elements[index].visible && question.elements[index].id == optionId)
					{
						return question.elements[index];
					}
				}

				return null;
			}

			this.restore = function ()
			{
				for (var index = 0; index <= lastRadioIndex; ++index)
				{
					question.elements[index].checked = states[index];
				}

				if (hasOptionalTextBox)
				{
					optionalTextBox.value = states[states.length - 1].value;
					optionalTextBox.disabled = states[states.length - 1].disabled;
				}
			}
		},

		ghrOperator: function (question)
		{
			var Utils = MS.Support.Fms.Utils;

			var ranking = Utils.getChildByName(question.domObject, "ranking");
			var isRanking = ranking && ranking.value == "1";

			var table = question.domObject.getElementsByTagName("table")[0];
			var labelRow = table.rows[0];
			var headerRow = table.rows[1];

			var ghrRows = Utils.findChildNodes(question.domObject,
				function (e)
				{
					return Utils.hasClass(e, "GHR_ODDROW") || Utils.hasClass(e, "GHR_EVENROW");
				}
			);

			var columns = headerRow.cells.length - 1;
			var rows = ghrRows.length;

			var hasDontKnow = question.elements[0].value == "0" || question.elements[columns - 1].value == "0";

			if (hasDontKnow)
			{
				var dontKnowColumnIndex = question.elements[0].value == "0" ? 0 : columns - 1;
			}

			var autoHideColumns = false;
			// for ranking question, if row count equals to column count (exclude don't know), then columns shall be hide automatically
			if (isRanking && ((hasDontKnow && columns == rows + 1) || ((!hasDontKnow) && columns == rows)))
			{
				autoHideColumns = true;
			}

			var rankingDirection = "asc";

			if (isRanking)
			{
				var index = (hasDontKnow && dontKnowColumnIndex == 0) ? 1 : 0;

				if ((index + 1) < columns && (question.elements[index] && question.elements[index + 1] && (question.elements[index].value > question.elements[index + 1].value)))
				{
					rankingDirection = "desc";
				}
			}

			var visibleRowCount = rows;
			var visibleColumnCount = columns;

			this.validateRequired = function ()
			{
				var count = 0;
				for (var index = 0; index < question.elements.length; ++index)
				{
					var e = question.elements[index];
					if (e.visible && e.checked)
					{
						++count;
					}
				}

				if (isRanking && count == Math.min(visibleRowCount, visibleColumnCount) || !isRanking && count == visibleRowCount)
				{
					return true;
				}

				if (question.elements.length == 0)
				{
					return true;
				}

				return false;
			}

			this.validateInput = function ()
			{
				return new MS.Support.Fms.ValidateResult(true);
			}

			function handleRankingRadioClick(rowIndex, columnIndex)
			{
				if (hasDontKnow && columnIndex == dontKnowColumnIndex)
				{
					return; // skip don't known
				}

				var index = rowIndex * columns + columnIndex;
				for (var i = 0; i < rows; ++i)
				{
					var elementIndex = i * columns + columnIndex;

					if (elementIndex != index)
					{
						question.elements[elementIndex].checked = false;
					}

				}
			}

			this.prepare = function ()
			{
				visibleRowCount = 0;
				var staticOptionCount = 0;
				for (var index = 0; index < ghrRows.length; ++index)
				{
					var row = ghrRows[index];

					var showIf = true;
					var visibility = row.getAttribute("visibility");
					var retVal = question.survey.EvaluateContextVisibility(row);

					switch (visibility)
					{
						case "Show":
							showIf = retVal == "NotExist" ? false : retVal == "EvalTrue" ? true : false;
							break;
						case "Hide":
							showIf = retVal == "NotExist" ? true : retVal == "EvalTrue" ? false : true;
							break;
						case "Selected":
							//Not implement selected logic in GHR/SHR option currently.
						default:
							break;
					}

					if (showIf)
					{
						++visibleRowCount;
						if (question.isStaticOption(row))
						{
							++staticOptionCount;
						}

						Utils.removeClass(row, "GHR_EVENROW GHR_ODDROW");
						Utils.addClass(row, visibleRowCount % 2 == 0 ? "GHR_EVENROW" : "GHR_ODDROW");
						Utils.displayTableRow(row);
					}
					else
					{
						Utils.hideElement(row);
					}

					row.visible = showIf;

					var startJ = index * columns, endJ = startJ + columns;
					for (var j = startJ; j < endJ; ++j)
					{
						question.elements[j].visible = showIf;
					}
				}

				if (autoHideColumns && visibleRowCount > 0)
				{
					visibleColumnCount = Math.min(columns, visibleRowCount + (hasDontKnow ? 1 : 0));
					var cellWidth = (0.75 / visibleColumnCount * 100) + "%";
					var firstRankingColumnIndex = (hasDontKnow && dontKnowColumnIndex == 0) ? 1 : 0;
					var rankingColumnCount = columns - (hasDontKnow ? 1 : 0);
					var visibleRankingColumnCount = visibleColumnCount - (hasDontKnow ? 1 : 0);

					for (index = firstRankingColumnIndex; index < rankingColumnCount; ++index)
					{
						if ((rankingDirection == "asc" && index < visibleRankingColumnCount) || (rankingDirection == "desc" && index >= (rankingColumnCount - visibleRankingColumnCount)))
						{
							Utils.displayTableCell(labelRow.cells[index + 1]);
							Utils.displayTableCell(headerRow.cells[index + 1]);
							headerRow.cells[index].width = cellWidth;
						}
						else
						{
							Utils.hideElement(labelRow.cells[index + 1]);
							Utils.hideElement(headerRow.cells[index + 1]);
						}
					}

					for (var index = 0; index < ghrRows.length; ++index)
					{
						var row = ghrRows[index];
						if (row.visible)
						{
							for (var j = firstRankingColumnIndex; j < rankingColumnCount; ++j)
							{
								if ((rankingDirection == "asc" && j < visibleRankingColumnCount) || (rankingDirection == "desc" && j >= (rankingColumnCount - visibleRankingColumnCount)))
								{
									Utils.displayTableCell(row.cells[j + 1]);
									question.elements[index * columns + j].visible = true;
								}
								else
								{
									Utils.hideElement(row.cells[j + 1]);
									question.elements[index * columns + j].visible = false;
								}
							}
						}
					}
				}

				if (isRanking && (visibleRowCount <= 1 && staticOptionCount == 0) || !isRanking && (visibleRowCount + staticOptionCount == 0))
				{
					question.needHide = true;
				}
			}

			this.populateVariables = function ()
			{
				var variables = [];

				var questionVariable = getQuestionVariable();
				if (questionVariable != null)
				{
					variables[questionVariable.name] = questionVariable.value;
				}
				var optionVariable = getOptionVariable();
				if (optionVariable != null)
				{
					for (var index = 0; index < optionVariable.length; ++index)
					{
						var variable = optionVariable[index];
						variables[variable.name] = variable.value;
					}
				}

				return variables;
			}

			function getQuestionVariable() 
			{
				var variable = question.domObject.getAttribute("var");

				if (variable != null && variable != "")
				{
					var value = "";
					for (var r = 0; r <= rows; r++)
					{
						for (var c = 0; c < columns; c++)
						{
							var e = question.elements[r * columns + c];
							if (e && e.visible && e.checked)
							{
								value += ", " + e.id.toString() + "|" + e.value.toString();
							}
						}
					}
					return { name: variable, value: (value.length > 2 ? value.substring(2) : null) };
				}
				return null;
			}

			function getOptionVariable()
			{
				var variableList = [];

				for (var row = 0; row < ghrRows.length; ++row)
				{
					var variableName = ghrRows[row].getAttribute("var");
					if (!variableName)
					{
						continue;
					}
					for (var col = 0; col < columns; ++col)
					{
						var e = question.elements[row * columns + col];
						if (e.visible && e.checked)
						{
							variableList.push({ name: variableName, value: e.getAttribute("value") });
						}
					}
				}

				return variableList;
			}

			this.getOptionById = function (optionId)
			{
				for (var i = 0; i < question.elements.length; ++i)
				{
					var element = question.elements[i];
					if (element.visible && element.id == optionId && element.checked)
					{
						return element;
					}
				}

				return null;
			}

			function isFixed(element)
			{
				return (element.getAttribute("fixed") == "1");
			}

			this.randomize = function ()
			{
				if (question.needRandomization())
				{
					var randomizedOptions = [];
					var nonFixedOptions = [];

					for (var index = 0; index < ghrRows.length; ++index)
					{
						if (isFixed(ghrRows[index]))
						{
							randomizedOptions[index] = ghrRows[index];
						}
						else
						{
							nonFixedOptions.push(index);
						}
					}

					Utils.randomize(nonFixedOptions);

					for (var index = 0; index < ghrRows.length; ++index)
					{
						if (randomizedOptions[index] == null)
						{
							randomizedOptions[index] = ghrRows[nonFixedOptions.shift()];
						}
					}

					for (var i = 0; i < ghrRows.length; ++i)
					{
						if (randomizedOptions[i] != ghrRows[i])
						{
							for (var j = 0; j < ghrRows.length; ++j)
							{
								if (ghrRows[j] == randomizedOptions[i])
								{
									Utils.swapNode(ghrRows[i], randomizedOptions[i]);

									ghrRows[j] = ghrRows[i];
									ghrRows[i] = randomizedOptions[i];

									for (var k = 0; k < columns; ++k)
									{
										var t = question.elements[i * columns + k];
										question.elements[i * columns + k] = question.elements[j * columns + k];
										question.elements[j * columns + k] = t;
									}

									break;
								}
							}
						}
					}
				}

				if (isRanking)
				{
					for (var index = 0; index < question.elements.length; ++index)
					{
						(function ()
						{
							var columnIndex = index % columns;
							var rowIndex = Math.floor(index / columns);

							Utils.addEventHandler(question.elements[index], "click", function () { handleRankingRadioClick(rowIndex, columnIndex); return true; });
						})();
					}
				}
			}

			this.getAnswers = function ()
			{
				var answers = [];

				for (var index = 0; index < question.elements.length; ++index)
				{
					var e = question.elements[index];
					if (e.visible && e.checked)
					{
						var id = e.id, value = e.value;
						answers.push({ id: id, value: value, text: "" });
					}
				}

				return answers;
			}

			var states = null;

			this.save = function ()
			{
				states = [];

				for (var index = 0; index < question.elements.length; ++index)
				{
					states.push(question.elements[index].checked);
				}
			}

			this.restore = function ()
			{
				for (var index = 0; index < question.elements.length; ++index)
				{
					question.elements[index].checked = states[index];
				}
			}
		},

		choicelistOperator: function (question)
		{
			var Utils = MS.Support.Fms.Utils;

			var selectObject = question.elements[0];
			selectObject.selectedIndex = selectObject.multiple ? -1 : 0;
			var options = selectObject.options;

			var allOptions = [];

			for (var index = 0; index < options.length; ++index)
			{
				allOptions.push({ option: options[index], selected: options[index].selected });
			}

			this.validateRequired = function ()
			{
				if (options.length == 0 || options.length == 1 && options[0].value == -1)
				{
					//drop down single select which has only one non-select item.
					return true;
				}
				for (var index = 0; index < options.length; ++index)
				{
					var option = options[index];
					if (option.selected && option.id != "" && option.value != "" && option.value != -1)
					{
						return true;
					}
				}
				return false;
			}

			this.validateInput = function ()
			{
				return new MS.Support.Fms.ValidateResult(true);
			}

			this.prepare = function ()
			{
				for (var index = 0; index < allOptions.length; ++index)
				{
					allOptions[index].selected = allOptions[index].option.selected;
				}

				selectObject.options.length = 0;

				for (var index = 0; index < allOptions.length; ++index)
				{
					var bShow = true, bSelect = allOptions[index].option.selected;
					var visibility = allOptions[index].option.getAttribute("visibility");


					var retVal = question.survey.EvaluateContextVisibility(allOptions[index].option);

					switch (visibility)
					{
						case "Show":
							bShow = retVal == "NotExist" ? false : retVal == "EvalTrue" ? true : false;
							break;
						case "Hide":
							bShow = retVal == "NotExist" ? true : retVal == "EvalTrue" ? false : true;
							break;
						case "Selected":
							bShow = true;
							bSelect = retVal == "NotExist" ? false : retVal == "EvalTrue" ? true : false;
							break;
						default:
							break;
					}
					if (bShow)
					{
						selectObject.options.add(allOptions[index].option);
						if (bSelect)
						{
							allOptions[index].option.selected = true;
						}
						else
						{
							allOptions[index].option.selected = false;
						}
					}
				}

				if (selectObject.options.length == 0 || options.length == 1 && options[0].value == -1)
				{
					question.needHide = true;
				}
			}

			function getOptionText(e)
			{
				return e.selected ? e.text : null;
			}

			function getOptionVariable(index)
			{
				var e = options[index];

				var variable = e.getAttribute("var");

				if (variable != null && variable != "")
				{
					return { name: variable, value: getOptionText(e) };
				}

				return null;
			}

			function getQuestionVariable()
			{
				var variable = question.domObject.getAttribute("var");

				if (variable != null && variable != "")
				{
					var value = "";
					for (var index = 0; index < options.length; ++index)
					{
						var e = options[index];
						if (e.selected)
						{
							value += "," + getOptionText(e);
						}
					}
					return { name: variable, value: value.length > 1 ? value.substring(1) : null };
				}

				return null;
			}

			this.populateVariables = function ()
			{
				var variables = [];

				var questionVariable = getQuestionVariable();
				if (questionVariable != null)
				{
					variables[questionVariable.name] = questionVariable.value;
				}

				for (var index = 0; index < options.length; ++index)
				{
					var optionVariable = getOptionVariable(index);

					if (optionVariable != null)
					{
						variables[optionVariable.name] = optionVariable.value;
					}
				}

				return variables;
			}

			this.getOptionById = function (optionId)
			{
				for (var i = 0; i < question.elements.length; ++i)
				{
					var element = question.elements[i];
					for (var j = 0; j < element.options.length; ++j)
					{
						if (element.options[j].id == optionId)
						{
							return element.options[j];
						}
					}
				}

				return null;
			}

			function isFixed(option)
			{
				return (option.value == "-1" || option.getAttribute("fixed") == "1");
			}

			this.randomize = function ()
			{
				if (question.needRandomization())
				{
					var randomizedOptions = [];
					var nonFixedOptions = [];

					for (var index = 0; index < allOptions.length; ++index)
					{
						if (isFixed(allOptions[index].option))
						{
							randomizedOptions[index] = allOptions[index];
						}
						else
						{
							nonFixedOptions.push(allOptions[index]);
						}
					}

					Utils.randomize(nonFixedOptions);

					for (var index = 0; index < allOptions.length; ++index)
					{
						if (randomizedOptions[index] == null)
						{
							allOptions[index] = nonFixedOptions.shift();
						}
					}
				}
			}

			this.getAnswers = function ()
			{
				var answers = [];

				for (var index = 0; index < options.length; ++index)
				{
					var option = options[index];
					if (option.selected && option.id != "" && option.value != "" && option.value != -1)
					{
						answers.push({ id: option.id, value: option.value, text: option.text ? option.text : "" });
					}
				}

				return answers;
			}

			var states = null;

			this.save = function ()
			{
				states = [];

				states.selectedIndex = selectObject.selectedIndex;

				for (var index = 0; index < allOptions.length; ++index)
				{
					states.push(allOptions[index].option.selected);
				}
			}

			this.restore = function ()
			{
				selectObject.selectedIndex = states.selectedIndex;

				for (var index = 0; index < allOptions.length; ++index)
				{
					allOptions[index].option.selected = states[index];
				}
			}
		}
	};

	MS.Support.Fms.BranchRule = function (optionId, optionValue, target)
	{
		this.optionId = optionId;
		this.optionValue = optionValue;
		this.target = target;

		this.match = function (element)
		{
			if ((element.visible && element.checked) || element.selected)
			{
				if (typeof (optionId) != "undefined" && optionId != null)
				{
					if (element.id == optionId)
					{
						if (typeof (optionValue) != "undefined" && optionValue != null)
						{
							if (element.value == optionValue)
							{
								return target;
							}
						}
						else
						{
							return target;
						}
					}
				}
				else
				{
					return target;
				}
			}
			else if (element.options != null)
			{
				for (var index = 0; index < element.options.length; ++index)
				{
					if (target == this.match(element.options[index]))
					{
						return target;
					}
				}
			}

			return -1;
		}
	}

	MS.Support.Fms.Condition = function (pageId, questionId, optionId, optionValue)
	{
		this.getPageId = function ()
		{
			return pageId;
		}

		this.getQuestionId = function ()
		{
			return questionId;
		}

		this.getOptionId = function ()
		{
			return optionId;
		}

		this.getOptionValue = function ()
		{
			return optionValue;
		}

		this.evaluate = function (survey)
		{
			var page = survey.getPageById(pageId);
			if (page && page.modified)
			{
				var question = page.getQuestionById(questionId);
				if (question)
				{
					var option = question.getOptionById(optionId);
					if (option)
					{
						if (((option.visible && option.checked) && ((question.getQuestionType() != "CHOICE-SEQ" && question.getQuestionType() != "CHOICE-HOR") || option.value == optionValue)) || option.selected)
						{
							return true;
						}
					}
				}
			}

			return false;
		}
	}

	MS.Support.Fms.ContextBranchRuleCondition = function (contextName, comparisonOperator, expectedValue)
	{
		this.contextName = contextName;
		this.comparisonOperator = comparisonOperator;
		this.expectedValue = expectedValue;

		if (typeof MS.Support.Fms.ContextBranchRuleCondition.evaluators == "undefined")
		{
			MS.Support.Fms.ContextBranchRuleCondition.createRegexp = function (str)
			{
				var i = str.lastIndexOf("/");
				var re = null;
				try
				{
					re = new RegExp(str.substring(1, i), str.substring(i + 1));
				}
				catch (err)
				{
					return null;
				}

				return re;
			};

			MS.Support.Fms.ContextBranchRuleCondition.evaluators = {
				"=": function (val, expectedValue)
				{
					return val.toString() === expectedValue;
				},
				"<>": function (val, expectedValue)
				{
					return val.toString() !== expectedValue;
				},
				">": function (val, expectedValue)
				{
					return val.toString() > expectedValue;
				},
				"<": function (val, expectedValue)
				{
					return val.toString() < expectedValue;
				},
				">=": function (val, expectedValue)
				{
					return val.toString() >= expectedValue;
				},
				"<=": function (val, expectedValue)
				{
					return val.toString() <= expectedValue;
				},
				"Contains": function (val, expectedValue)
				{
					return val.toString().indexOf(expectedValue) !== -1;
				},
				"Not Contain": function (val, expectedValue)
				{
					return val.toString().indexOf(expectedValue) === -1;
				},
				"Starts With": function (val, expectedValue)
				{
					return val.toString().indexOf(expectedValue) === 0;
				},
				"Not Start With": function (val, expectedValue)
				{
					return val.toString().indexOf(expectedValue) !== 0;
				},
				"Ends With": function (val, expectedValue)
				{
					val = val.toString();
					var i = val.lastIndexOf(expectedValue);
					return i !== -1 && i === (val.length - expectedValue.length);
				},
				"Not End With": function (val, expectedValue)
				{
					val = val.toString();
					var i = val.lastIndexOf(expectedValue);
					return i === -1 || val.lastIndexOf(expectedValue) !== (val.length - expectedValue.length);
				},
				"Matches": function (val, expectedValue)
				{
					var re = MS.Support.Fms.ContextBranchRuleCondition.createRegexp(expectedValue);
					return re ? re.test(val.toString()) : false;
				},
				"Not Match": function (val, expectedValue)
				{
					var re = MS.Support.Fms.ContextBranchRuleCondition.createRegexp(expectedValue);
					return re ? !(re.test(val.toString())) : false;
				},
				"Exists": function (val, expectedValue)
				{
					return typeof val != 'undefined' && val != null;
				},
				"Not Exist": function (val, expectedValue)
				{
					return typeof val == 'undefined' || val == null;
				}
			};
		}

		this.evaluate = function (context)
		{
			if (context == null)
			{
				return false;
			}

			var val = context[this.contextName];
			if ((typeof val == "undefined" || val == null) && this.comparisonOperator != "Exists" && this.comparisonOperator != "Not Exist")
			{
				return false;
			}

			var evaluator = MS.Support.Fms.ContextBranchRuleCondition.evaluators[this.comparisonOperator];
			return evaluator ? evaluator(val, this.expectedValue) : false;
		}
	}

	MS.Support.Fms.CompoundingBranchRule = function (nextPageId, probability)
	{
		this.conditions = [];
		this.contextConditions = [];
		this.nextPageId = nextPageId;
		this.probability = probability;

		this.addConditions = function (cbrs)
		{
			for (var n = 0; n < cbrs.length - 1; n++)
			{
				var datas = cbrs[n].split("-");
				var sourcePageId = parseInt(datas[0]);
				var questionId = parseInt(datas[1]);
				var optionId = parseInt(datas[2]);
				var optionValue = parseInt(datas[3]);
				this.conditions.push(new MS.Support.Fms.Condition(sourcePageId, questionId, optionId, optionValue));
			}

		}

		this.addContextConditions = function (cnbrs)
		{
			for (var n = 0; n < cnbrs.length; n++)
			{
				var cnbr = cnbrs[n];
				var contextName = cnbr.getAttribute('data-cname');
				var comparisionOperator = cnbr.getAttribute('data-coper');
				var expectedValue = cnbr.getAttribute('data-cval');
				this.contextConditions.push(new MS.Support.Fms.ContextBranchRuleCondition(contextName, comparisionOperator, expectedValue));
			}
		}

		this.evaluate = function (survey, context)
		{
			for (var i = 0; i < this.conditions.length; ++i)
			{
				if (!this.conditions[i].evaluate(survey))
				{
					return false;
				}
			}

			for (var i = 0; i < this.contextConditions.length; ++i)
			{
				if (!this.contextConditions[i].evaluate(context))
				{
					return false;
				}
			}
			// if hit the probability, return true
			// else return false
			if (isNaN(this.probability) || this.probability < 0)
			{
				return true;
			}
			else if (this.probability > 0 && Math.random() * 100 <= this.probability)
			{
				return true;
			}
			else
			{
				return false;
			}
			return true;
		}
	}

	MS.Support.Fms.ExpressionWrapper = function (expression, survey)
	{
		var variables = survey.getVariables();
		var context = survey.getContext();
		var urlParameters = survey.urlParameters;

		var reFunc = /((AND|OR|NOT|Match|getVariable|getContext|GreaterThan|LessThan|GreaterThanOrEqual|LessThanOrEqual|Equal)\()/g;
		var reVar = /({\@[^}]+})/gi;
		var reContext = /({\$[^}]+})/gi;
		var tagStart = "{";
		var tagEnd = "}";
		var varPrefix = "@";
		var contextPrefix = "$";
		var parameterPrefix = "$Url:";

		this.encodeValue = function (value)
		{
			if (typeof (value) == "undefined" || value == null)
			{
				return value;
			} else
			{
				return value.replace(/&/g, "&amp;").replace(/}/g, "&rb;").replace(/{/g, "&lb;");
			}
		}

		this.decodeValue = function (value)
		{
			if (typeof (value) == "undefined" || value == null)
			{
				return value;
			} else
			{
				return value.replace(/&rb;/g, "}").replace(/&lb;/g,"{").replace(/&amp;/g, "&");
			}
		}

		this.getVariable = function (variableName)
		{
			//variableName format: {@abc}
			variableName = variableName.replace(/{\@([^}]+)}/gi, "$1");
			return this.encodeValue(variables[variableName]);
		}

		this.getContext = function (contextName)
		{
			// contextName format: {$abc"}
			// urlParameterName format: {$Url:abc}
			//Decode the contextName if there is some encoded tag symbols "{"| "}"
			if (contextName.substring(1, parameterPrefix.length + 1) == parameterPrefix)
			{
				contextName = contextName.replace(/{\$Url:([^}]+)}/gi, "$1").replace(/\[0x7b\]/g, "{").replace(/\[0x7d\]/g, "}").toLowerCase();
				return this.encodeValue(urlParameters[contextName]);
			}
			else
			{
			contextName = contextName.replace(/{\$([^}]+)}/gi, "$1").replace(/\[0x7b\]/g, "{").replace(/\[0x7d\]/g, "}");
				return this.encodeValue(context[contextName]);
		}
		}

		//Compatible with old survey xml format.
		this.isDefined = function (name)
		{
			return this.getVariable(name) == null ? false : true;
		}

		this.Match = function (name, pattern)
		{
			return pattern.test(name);
		}


		this.GreaterThan = function (leftValue, rightValue)
		{
			return leftValue > rightValue;
		}

		this.LessThan = function (leftValue, rightValue)
		{
			return leftValue < rightValue;
		}


		this.GreaterThanOrEqual = function (leftValue, rightValue)
		{
			return leftValue >= rightValue;
		}


		this.LessThanOrEqual = function (leftValue, rightValue)
		{
			return leftValue <= rightValue;
		}


		this.Equal = function (leftValue, rightValue)
		{
			return leftValue == rightValue;
		}

		this.AND = function ()
		{
			for (var index = 0; index < arguments.length; ++index)
			{
				if (!arguments[index])
				{
					return false;
				}
			}
			return true;
		}

		this.OR = function ()
		{
			for (var index = 0; index < arguments.length; ++index)
			{
				if (arguments[index])
				{
					return true;
				}
			}
			return false;
		}

		this.NOT = function (result)
		{
			return !result;
		}

		this.Trim = function (str)
		{
			return str.replace(/(^\s*)|(\s*$)/g, "").replace(/\r/g, "").replace(/\n/g, "").replace(/\t/g, "");
		}

		this.replaceStartEndTag = function (str)
		{
			var escapeStartTag = "[0x7b]", escapeEndTag = "[0x7d]";
			var startIndex = str.indexOf("{@");
			if (startIndex < 0)
			{
				startIndex = str.indexOf("{$");
			}

			var endIndex = str.lastIndexOf(tagEnd);

			if (startIndex < 0 || endIndex < 0)
			{
				return str;
			}
			var start = str.substring(0, startIndex + 2);
			var end = str.substring(endIndex);

			var tagContent = str.substring(startIndex + 2, endIndex);
			tagContent = tagContent.replace(/{/g, escapeStartTag).replace(/}/g, escapeEndTag);
			if (tagContent == null || tagContent == "")
			{
				return "";
			}
			return start + tagContent + end;
		}

		this.filterInvalidCharacters = function (expression)
		{
			var innerStack = [];
			var index = 0, temp = null;
			var strRet = "";

			while (index < expression.length)
			{
				temp = expression.charAt(index);
				switch (temp)
				{
					case tagStart:
					case tagEnd:
					default:
						innerStack.push(temp);
						break;
					case varPrefix:
					case contextPrefix:
					case parameterPrefix:
						if (innerStack[innerStack.length - 1] == tagStart)
						{
							innerStack[innerStack.length - 1] += temp;
							if (innerStack.length > 1)
							{
								strRet += this.replaceStartEndTag(innerStack.slice(0, innerStack.length - 1).join(""));
							}
							innerStack = innerStack.slice(innerStack.length - 1);
						}
						else
						{
							innerStack.push(temp);
						}
						break;
				}
				index++;
			}
			if (innerStack.length > 0)
			{
				strRet += this.replaceStartEndTag(innerStack.join(""));
			}
			return strRet;
		}

		

		expression = this.filterInvalidCharacters(this.Trim(expression));
		expression = expression.replace(reFunc, "this.$1");
		expression = expression.replace(reVar, "this.getVariable(\"$1\")");
		expression = expression.replace(reContext, "this.getContext(\"$1\")");

		this.Execute = new Function("return " + expression + ";");
	}

	MS.Support.Fms.ReplaceableTextNode = function (question, domObject)
	{
		this.question = question;
		this.survey = question.survey;
		this.domObject = domObject;

		function isTextEditor(e)
		{
			var tagName = (e && e.tagName) ? e.tagName.toLowerCase() : "";

			if (tagName == "textarea" || (tagName == "input" && (e.type.toLowerCase() == "text" || e.type.toLowerCase() == "number" || e.type.toLowerCase() == "date")))
			{
				return true;
			}

			return false;
		}

		function isTextElement(e)
		{
			var tagName = (e && e.tagName) ? e.tagName.toLowerCase() : "";

			if (tagName == "textarea" || (tagName == "input" && (e.type.toLowerCase() == "text" || e.type.toLowerCase() == "hidden" || e.type.toLowerCase() == "number" || e.type.toLowerCase() == "date")))
			{
				return true;
			}

			return false;
		}

		function isOption(e)
		{
			var tagName = (e && e.tagName) ? e.tagName.toLowerCase() : "";
			return tagName == "option";
		}

		function getDomValue(e)
		{
			return isTextElement(e) ? e.value : (isOption(e) ? e.text : e.innerHTML);
		}

		function setDomValue(e, value)
		{
			if (isTextElement(e))
			{
				e.value = value;
			}
			else if (isOption(e))
			{
				e.text = value;
			}
			else
			{
				e.innerHTML = value;
			}
		}

		var replaceOnce = isTextEditor(domObject);
		var replaceCount = 0;
		this.originalValue = getDomValue(domObject);
		this.variables = this.originalValue.match(new RegExp("{(\\$(Url:)?|@(Complex:\\s*)?)[^}]*}", "g"));

		this.replace = function ()
		{
			if (replaceOnce && replaceCount >= 1)
			{
				return;
			}

			var text = this.originalValue;

			for (var i = 0; i < this.variables.length; ++i)
			{
				var variable = this.variables[i];

				var variableContent = variable.substring(1, variable.length - 1);
				var variableValue = "";

				var contextPrefix = "$";
				var urlParameterPrefix = "$Url:";
				var complexExpPrefix = "@Complex:";

				if (variableContent.substring(0, complexExpPrefix.length) == complexExpPrefix)
				{
					var exp = variableContent.replace(complexExpPrefix, "").replace(/^\s*/, "")
					exp = exp.replace(/\[0x7b\]/g, "{").replace(/\[0x7d\]/g, "}");
					try
					{
						var expWrapper = new MS.Support.Fms.ExpressionWrapper(exp, this.survey);
						variableValue = expWrapper.Execute();
						if (typeof (variableValue) == "undefined" || variableValue == null)
						{
							variableValue = "";
						}

						variableValue = expWrapper.decodeValue(variableValue);
					}
					catch (e)
					{
						variableValue = "";
					}
				}
				else if (variableContent.substring(0, urlParameterPrefix.length) == urlParameterPrefix)
				{
					var urlParameters = this.survey.urlParameters;
					var exp = variableContent.replace(urlParameterPrefix, "");
					exp = exp.replace(/\[0x7b\]/g, "{").replace(/\[0x7d\]/g, "}").toLowerCase();
					variableValue = urlParameters ? urlParameters[exp] : "";
				}
				else if (variableContent.substring(0, contextPrefix.length) == contextPrefix)
				{
					var context = this.survey.getContext();
					var exp = variableContent.substring(1);
					exp = exp.replace(/\[0x7b\]/g, "{").replace(/\[0x7d\]/g, "}");
					variableValue = context ? context[exp] : "";
				}
				else
				{
					variableValue = this.survey.getVariable(variableContent.substring(1));
				}

				if (!variableValue)
				{
					variableValue = "";
				}

				if (isTextElement(this.domObject) || isOption(this.domObject))
				{
					variableValue = MS.Support.Fms.Utils.characterEncode(variableValue);
				}
				else
				{
					variableValue = MS.Support.Fms.Utils.characterEncode(MS.Support.Fms.Utils.htmlEncode(variableValue));
				}
				text = text.replace(variable, variableValue);
			}

			setDomValue(this.domObject, text);
			++replaceCount;
		}
	}

	MS.Support.Fms.SurveyQuestion = function (domObject, page)
	{
		var Fms = MS.Support.Fms;
		var Utils = Fms.Utils;
		var ValidateResult = Fms.ValidateResult;

		this.domObject = domObject;
		var type = Utils.getChildByName(domObject, "type");
		var randomization = Utils.getChildByName(domObject, "randomization");
		var operator = null;

		var branchRules = [];
		this.page = page;
		this.survey = page.survey;
		this.needHide = false;

		var visibility = this.domObject.getAttribute("visibility");

		this.getVisibility = function ()
		{
			return visibility;
		}

		function isBranchRuleNode(node)
		{
			return node && node.type && node.type.toLowerCase() == "hidden" && node.id.indexOf("NextRef_") == 0;
		}

		this.findBranchRules = function ()
		{
			if (this.domObject != null)
			{
				var es = Utils.findChildNodes(this.domObject, isBranchRuleNode);

				for (var index = 0; index < es.length; ++index)
				{
					var parts = es[index].id.split("_", 3);

					branchRules.push(new MS.Support.Fms.BranchRule(parts[1], parts[2], es[index].value));
				}
			}
		}

		this.findBranchRules();

		var requiredHidden = Utils.getChildByName(domObject, "required");
		var requiredErrorText = null;

		this.isRequired = function ()
		{
			return requiredHidden != null && requiredHidden.value == "1";
		}

		this.setRequired = function (value)
		{
			if (requiredHidden != null)
			{
				requiredHidden.value = value ? 1 : 0;
			}
		}

		if (this.isRequired())
		{
			requiredErrorText = Utils.getChildByName(domObject, "requiredErrorText");
		}

		this.evaluteShowIf = function (element)
		{
			var showIf = element.getAttribute("showIf");

			if (showIf == null || showIf == "")
			{
				return true;
			}
			else
			{
				return this.survey.evalute(showIf);
			}
		}

		this.isStaticOption = function (element)
		{
			var showIf = element.getAttribute("showIf");
			return (showIf == null || showIf == "") ? true : false;
		}

		var replaceableClassNames = ["SURVERINTROTEXT", "QUESTIONTEXT", "QUESTIONINSTRUCTION", "QUESTIONREQUIRED", "ANSWERTEXT", "OPTIONALTEXTBOXINSTRUCTION", "GHR_QUESTIONTEXT", "ANSWERBOX", "GHR_LEGENDTEXT"];
		var innerHTMLReadOnlyTags = ["table", "tbody", "tfoot", "thead", "tr"]; // There are also some other HTML elements whose innerHTML attributes are readonly on IE, but not likely will appear in our survey

		function isTextReplaceable(e)
		{
			if (e != null && e.tagName)
			{
				var replacePattern = new RegExp("{(\\$(Url:)?|#|@(Complex:\\s*)?)[^}]*}", "g");

				var tagName = e.tagName.toLowerCase();

				for (var j = 0; j < innerHTMLReadOnlyTags.length; ++j)
				{
					if (tagName == innerHTMLReadOnlyTags[j])
					{
						return false;
					}
				}

				if (tagName == "option")
				{
					return replacePattern.test(e.innerHTML);
				}
				else if (tagName == "textarea" || (tagName == "input" && (e.type.toLowerCase() == "text" || e.type.toLowerCase() == "hidden" || e.type.toLowerCase() == "number" || e.type.toLowerCase() == "date")))
				{
					return replacePattern.test(e.value);
				}
				else if (tagName == "td" && e.parentNode && Utils.hasClass(e.parentNode, "GHR_LEGENDTEXT"))
				{
					return replacePattern.test(e.innerHTML);
				}
				else if (e.className != null)
				{

					if (Utils.hasClass(e, "QUESTIONINSTRUCTION") && tagName == "font")
					{
						return false;
					}

					for (var index = 0; index < replaceableClassNames.length; ++index)
					{
						if (Utils.hasClass(e, replaceableClassNames[index]))
						{
							return replacePattern.test(e.innerHTML);
						}
					}
				}
			}
			return false;
		}

		this.replaceableNodes = [];

		this.findReplaceableNodes = function ()
		{
			var nodes = Utils.findChildNodes(this.domObject, isTextReplaceable);

			for (var index = 0; index < nodes.length; ++index)
			{
				var node = nodes[index];
				this.replaceableNodes.push(new MS.Support.Fms.ReplaceableTextNode(this, node));
			}
		}

		this.findReplaceableNodes();

		this.replaceTextNodes = function ()
		{
			for (var index = 0; index < this.replaceableNodes.length; ++index)
			{
				this.replaceableNodes[index].replace();
			}
		}

		this.getQuestionId = function ()
		{
			return this.domObject.id.replace("SURVEYQUESTION_", "");
		}

		this.getQuestionType = function ()
		{
			return type.value;
		}

		this.needRandomization = function ()
		{
			if (randomization && randomization.value == "1")
			{
				return true;
			}
			return false;
		}

		this.validate = function ()
		{
			if (this.needHide)
			{
				return new ValidateResult(true);
			}
			if (this.isRequired())
			{
				var isValid = operator.validateRequired(this);

				if (!isValid)
				{
					return new ValidateResult(false, this.getRequiredErrorText());
				}
			}

			return operator.validateInput();
		}

		this.configVisibility = function ()
		{
			var retVal = this.survey.EvaluateContextVisibility(this.domObject);

			switch (visibility)
			{
				case "Show":
					this.needHide = retVal == "NotExist" ? true : retVal == "EvalTrue" ? false : true;
					break;
				case "Hide":
					this.needHide = retVal == "NotExist" ? false : retVal == "EvalTrue" ? true : false;
					break;
				default:
					this.needHide = false;
					break;
			}
		}

		this.prepare = function ()
		{
			this.replaceTextNodes();
			this.configVisibility();
			operator.prepare();
			domObject.style.display = this.needHide ? "none" : "block";
		}

		this.populateVariables = function ()
		{
			return operator.populateVariables();
		}

		this.save = function ()
		{
			operator.save();
		}

		this.restore = function ()
		{
			operator.restore();
		}

		this.getAnswers = function ()
		{
			return operator.getAnswers();
		}

		this.getRequiredErrorText = function ()
		{
			if (!this.isRequired())
			{
				throw "Not a required question!";
			}
			else
			{
				return requiredErrorText.value;
			}
		}

		this.getNextPageByBranchRule = function ()
		{
			for (var i = 0; i < this.elements.length; ++i)
			{
				for (var j = 0; j < branchRules.length; ++j)
				{
					var target = branchRules[j].match(this.elements[i]);

					if (target != null && target != -1)
					{
						return target;
					}
				}
			}
			// -1 means not appcable
			return -1;
		}

		this.elements = [];

		function isQuestionElement(node)
		{
			if (node.type != null)
			{
				switch (node.type.toLowerCase())
				{
					case "text":
					case "date":
					case "password":
					case "textarea":
					case "number":
					case "radio":
					case "checkbox":
					case "select-one":
					case "select-multiple":
						return true;
				}
			}

			return false;
		}

		this.getOptionById = function (optionId)
		{
			return operator.getOptionById(optionId);
		}

		this.discoverElements = function ()
		{
			if (this.domObject != null)
			{
				var es = Utils.findChildNodes(this.domObject, isQuestionElement);

				for (var index = 0; index < es.length; ++index)
				{
					this.elements.push(es[index]);
				}
			}
		}

		this.discoverElements();

		operator = MS.Support.Fms.SurveyQuestionOperator.create(this);
		operator.randomize();

		this.displayBranchRules = function ()
		{
			for (var i = 0; i < this.elements.length; ++i)
			{
				var options = this.elements[i].options;
				if (options)
				{
					var selectVisual = "<ol>";

					for (var j = 0; j < options.length; ++j)
					{
						selectVisual += "<li>" + options[j].innerHTML;
						for (var k = 0; k < branchRules.length; ++k)
						{
							if (branchRules[k].optionId == options[j].id && (typeof (branchRules[k].optionValue == "undefined") || branchRules[k].optionValue == options[j].value))
							{
								selectVisual += "(--&gt;" + branchRules[k].target + ")";
								break;
							}
						}
						selectVisual += "</li>";
					}

					selectVisual += "</ol>";

					var info = document.createElement("span");
					info.style.color = "green";
					info.innerHTML = selectVisual;

					this.elements[i].parentNode.appendChild(info);
				}
				else
				{
					for (var j = 0; j < branchRules.length; ++j)
					{
						if (branchRules[j].optionId == this.elements[i].id && (typeof (branchRules[j].optionValue) == "undefined" || branchRules[j].optionValue == this.elements[i].value))
						{
							var info = document.createElement("font");
							info.color = "green";
							info.innerHTML = "(--&gt;" + branchRules[j].target + ")";

							this.elements[i].parentNode.appendChild(info);
						}
					}
				}
			}
		}
	}

	MS.Support.Fms.SurveyPage = function (domObject, survey)
	{
		var Fms = MS.Support.Fms;
		var Utils = Fms.Utils;
		var ValidateResult = Fms.ValidateResult;

		this.domObject = domObject;

		this.nextPageId = -1;
		this.modified = false;
		this.survey = survey;
		this.ignored = false;

		var nextPageByTopicRef = Utils.getChildByName(domObject, "NextSectionByTopicRef");

		if (nextPageByTopicRef && nextPageByTopicRef.value != null && nextPageByTopicRef.value != "")
		{
			this.nextPageId = parseInt(nextPageByTopicRef.value);
		}

		this.compoundingBranchRules = [];
		var data = Utils.getChildByName(domObject, "CompoundingBranchRules");
		if (data)
		{
			var containers = data.childNodes;
			for (var i = 0; i < containers.length; i++)
			{
				var brs = Utils.getChildByName(containers[i], "brs");
				var cnbrs = Utils.getChildNodesByAttribute(containers[i], "name", "cnbr");
				var ref = containers[i].getAttribute('data-ref');
				var probability = containers[i].getAttribute('data-probability');
				var rule = new MS.Support.Fms.CompoundingBranchRule(parseInt(ref), parseInt(probability));

				if (brs && brs.value)
				{
					rule.addConditions(brs.value.split("*"));
				}

				if (cnbrs && cnbrs.length > 0)
				{
					rule.addContextConditions(cnbrs);
				}

				this.compoundingBranchRules.push(rule);
			}
		}

		var pageId;

		if (this.domObject.id == "DIV_0")
		{
			pageId = "0";
		}
		else if (this.domObject.id == "DIV_CLOSE")
		{
			pageId = "End";
		}
		else
		{
			pageId = Utils.getChildByName(domObject, "id").value;
		}

		this.getPageId = function ()
		{
			return pageId;
		}

		this.questions = [];

		this.discoverQuestions = function ()
		{
			if (this.domObject != null)
			{
				var es = Utils.getChildNodesByClass(this.domObject, "QUESTIONCONTAINER");

				for (var index = 0; index < es.length; ++index)
				{
					this.questions.push(new MS.Support.Fms.SurveyQuestion(es[index], this));
				}
			}
		}

		this.discoverQuestions();

		this.isRequired = function ()
		{
			for (var index = 0; index < this.questions.length; ++index)
			{
				if (this.questions[index].isRequired() && !this.questions[index].needHide)
				{
					return true;
				}
			}

			return false;
		}

		this.save = function ()
		{
			for (var index = 0; index < this.questions.length; ++index)
			{
				this.questions[index].save();
			}
		}

		var variables = [];

		this.getVariables = function ()
		{
			return variables;
		}

		this.populateVariables = function ()
		{
			variables = [];

			for (var index = 0; index < this.questions.length; ++index)
			{
				var subVariables = this.questions[index].populateVariables();

				for (var key in subVariables)
				{
					variables[key] = subVariables[key];
				}
			}
		}

		this.restore = function ()
		{
			for (var index = 0; index < this.questions.length; ++index)
			{
				this.questions[index].restore();
			}
		}

		this.validate = function ()
		{
			for (var index = 0; index < this.questions.length; ++index)
			{
				var result = this.questions[index].validate();
				if (!result.isValid)
				{
					return result;
				}
			}

			return new ValidateResult(true);
		}

		this.getNextPageByBranchRule = function ()
		{
			var nextPageId = -1;

			// If click "Skip" button, don't check question branch rule in this page.
			if (this.modified)
			{
				for (var index = 0; index < this.questions.length; ++index)
				{
					if ((nextPageId = this.questions[index].getNextPageByBranchRule()) != -1)
					{
						return nextPageId;
					}
				}
			}

			var context = survey.getContext();
			for (var i = 0; i < this.compoundingBranchRules.length; i++)
			{
				if (this.compoundingBranchRules[i].evaluate(this.survey, context))
				{
					return this.compoundingBranchRules[i].nextPageId;
				}
			}

			return this.nextPageId;
		}

		this.getQuestionById = function (questionId)
		{
			for (var index = 0; index < this.questions.length; ++index)
			{
				if (this.questions[index].getQuestionId() == questionId)
				{
					return this.questions[index];
				}
			}

			return null;
		}

		this.getQuestionByElement = function (el)
		{
			for (var i = 0; i < this.questions.length; ++i)
			{
				if (Utils.isAncestorOf(this.questions[i].domObject, el))
				{
					return this.questions[i];
				}
			}

			return null;
		}

		this.show = function ()
		{
			var isIgnored = true;
			this.ignored = false;
			var questionCount = this.questions.length;
			for (var index = 0; index < this.questions.length; ++index)
			{
				var question = this.questions[index];
				question.prepare();
				if (!question.needHide)
				{
					isIgnored = false;
				}
			}

			if (questionCount > 0 && isIgnored)
			{
				this.ignored = true;
			}
			if (this.domObject != null)
			{
				this.enableButton("btnSkip", !this.isRequired());
				this.domObject.style.display = "block";
			}
		}

		this.enableButton = function (buttonDataName, isEnabled)
		{
			var button = Utils.getChildByAttribute(this.domObject, "data-name", buttonDataName);
			if (!button)
			{
				return;
			}
			if (isEnabled)
			{
				button.removeAttribute("disabled");
			}
			else
			{
				button.setAttribute("disabled", "disabled");
			}
		}

		this.hide = function ()
		{
			if (this.domObject != null)
			{
				this.domObject.style.display = "none";
			}
		}

		this.isVisible = function ()
		{
			if (this.domObject != null)
			{
				return this.domObject.style.display == "none" ? false : true;
			}

			return false;
		}

		this.displayBranchRules = function (nextPageIdInOrder)
		{
			var nextPageId;
			if (this.nextPageId > 0)
			{
				nextPageId = this.nextPageId;
			}
			else
			{
				nextPageId = nextPageIdInOrder;
			}

			var info = document.createElement("span");
			info.style.color = "green";
			info.innerHTML = "(" + this.getPageId() + "--&gt;" + nextPageId + ")";

			if (this.domObject != null)
			{
				this.domObject.parentNode.insertBefore(info, this.domObject);
			}

			for (var index = 0; index < this.questions.length; ++index)
			{
				this.questions[index].displayBranchRules();
			}
		}
	}

	MS.Support.Fms.FollowUp = function (surveyIds, language, startOffset, endOffset, serverCoordinate, clientCoordinate, quitMode, trigger, conditions)
	{
		this.getSurveyIds = function ()
		{
			return surveyIds;
		}

		this.getLanguage = function ()
		{
			return language;
		}

		this.getStartOffset = function ()
		{
			return startOffset;
		}

		this.getEndOffset = function ()
		{
			return endOffset;
		}

		this.getQuitMode = function ()
		{
			return quitMode;
		}

		this.getTrigger = function ()
		{
			return trigger;
		}

		this.getConditions = function ()
		{
			return conditions;
		}

		this.getServerCoordinate = function ()
		{
			return serverCoordinate;
		}

		this.getClientCoordinate = function ()
		{
			return clientCoordinate;
		}

		this.getStartTime = function ()
		{
			return startOffset + clientCoordinate;
		}

		this.getEndTime = function ()
		{
			return endOffset + clientCoordinate;
		}

		this.isActive = function ()
		{
			var time = Math.round((new Date()).getTime() / 1000);
			return (time >= this.getStartTime() && time <= this.getEndTime());
		}

		this.isExpired = function ()
		{
			return Math.round((new Date()).getTime() / 1000) > this.getEndTime();
		}

		this.evalute = function (survey, isPartial)
		{
			if ((quitMode != 2) && (quitMode != (isPartial ? 1 : 0)))
			{
				return false;
			}

			for (var i = 0; i < conditions.length; ++i)
			{
				if (!conditions[i].evalute(survey))
				{
					return false;
				}
			}

			return true;
		}
	}

	MS.Support.Fms.FollowUp.parseFollowupsField = function (fieldValue, language, loadTime)
	{
		var followUps = [];
		var re = /^(\[\[(\d+)(,\d+)*\],\d+,\d+,\d+,\d,\d+,\[\[\d+,\d+,\d+,\d+\](,\[\d+,\d+,\d+,\d+\])*\]\](,\[\[(\d+)(,\d+)*\],\d+,\d+,\d+,\d,\d+,\[\[\d+,\d+,\d+,\d+\](,\[\d+,\d+,\d+,\d+\])*\]\])*)?$/;
		if (re.test(fieldValue))
		{
			var followUpMetas = eval('[' + fieldValue + ']'); // as we have verfied the fieldValue with a strict regular expression, this "eval" shoud be safe	

			var time = Math.round((new Date()).getTime() / 1000);
			var serverTimeOffset = time - loadTime;

			for (var i = 0; i < followUpMetas.length; ++i)
			{
				var followUpMeta = followUpMetas[i];

				var conditions = [];
				for (var j = 0; j < followUpMeta[6].length; ++j)
				{
					var conditionMeta = followUpMeta[6][j];
					conditions.push(new MS.Support.Fms.Condition(conditionMeta[0], conditionMeta[1], conditionMeta[2], conditionMeta[3]));
				}
				followUps.push(new MS.Support.Fms.FollowUp(followUpMeta[0], language, followUpMeta[1], followUpMeta[2], followUpMeta[3] + serverTimeOffset, time, followUpMeta[4], followUpMeta[5], conditions));
			}
		}

		return followUps;
	}

	MS.Support.Fms.FollowUp.parseFollowupsCookie = function (cookieValue)
	{
		var followUps = [];

		var re = /^(\[\[(\d+)(,\d+)*\],\'[a-zA-Z]+(-[a-zA-Z]+)?\',\d+,\d+,\d+,\d+,\d+\](,\[\[(\d+)(,\d+)*\],\'[[a-zA-Z]+(-[a-zA-Z]+)?\',\d+,\d+,\d+,\d+,\d+\])*)?$/;
		if (re.test(cookieValue))
		{
			var metas = eval('[' + cookieValue + ']'); // as we have verfied the cookieValue with a strict regular expression, this "eval" shoud be safe

			for (var i = 0; i < metas.length; ++i)
			{
				var meta = metas[i];
				followUps.push(new MS.Support.Fms.FollowUp(meta[0], meta[1], meta[2], meta[3], meta[4], meta[5], 2, meta[6], null));
			}
		}

		return followUps;
	}

	MS.Support.Fms.FollowUp.packageFollowupsCookie = function (followUps)
	{
		var values = [];
		for (var i = 0; i < followUps.length; ++i)
		{
			var followUp = followUps[i];
			values.push(
				"["
					+ "[" + followUp.getSurveyIds() + "]" + ","
					+ "'" + followUp.getLanguage() + "'" + ","
					+ followUp.getStartOffset() + ","
					+ followUp.getEndOffset() + ","
					+ followUp.getServerCoordinate() + ","
					+ followUp.getClientCoordinate() + ","
					+ followUp.getTrigger() +
				"]"
			);
		}

		return values.join();
	}

	MS.Support.Fms.FollowUp.getMaxFollowupsExpireTime = function (followUps)
	{
		var maxValue = 0;

		for (var i = 0; i < followUps.length; ++i)
		{
			var endTime = followUps[i].getEndTime();
			if (endTime > maxValue)
			{
				maxValue = endTime;
			}
		}

		var time = new Date();
		time.setTime(maxValue * 1000);

		return time;
	}

	MS.Support.Fms.SurveyExpression = function (survey, literal)
	{
		this.evalute = function ()
		{
			var f = new Function("return " + literal);
			return f.apply(survey);
		}
	}

	MS.Support.Fms.Survey = function (id, config)
	{
		// using
		var Fms = MS.Support.Fms;
		var Utils = Fms.Utils;
		var Event = Fms.SurveyEvent;
		var Page = Fms.SurveyPage;
		var FollowUp = Fms.FollowUp;

		this.cookieDomain = config ? config.site.cookieDomain : typeof (gCookieDomain) != "undefined" ? (gCookieDomain || document.domain) : document.domain;

		this.id = id;

		this.surveyTrackingText = null;

		var loadTime = Math.round((new Date()).getTime() / 1000);

		this.domObject = document.getElementById(id);
		this.name = Utils.getChildById(this.domObject, "surveyname").value;

		this.introduction = null;

		this.pages = [];
		this.thankyou = null;

		this.submitFields = [];

		var hasSubmitted = false;
		this.startTime = null;

		// Events
		this.onBeforeStart = new Event();
		this.onBeforeAccept = new Event();
		this.onBeforeDecline = new Event();
		this.onBeforeCancel = new Event();
		this.onBeforeNext = new Event();
		this.onBeforePrevious = new Event();
		this.onBeforeSkip = new Event();
		this.onBeforeClose = new Event();
		this.onBeforeSubmit = new Event();

		this.onAfterStart = new Event();
		this.onAfterAccept = new Event();
		this.onAfterDecline = new Event();
		this.onAfterCancel = new Event();
		this.onAfterNext = new Event();
		this.onAfterPrevious = new Event();
		this.onAfterSkip = new Event();
		this.onAfterClose = new Event();
		this.onAfterSubmit = new Event();

		this.onValidateError = new Event();

		this.getTrackingText = function ()
		{
			if (config && config.triggerConfig && config.triggerConfig.entity)
			{
				this.surveyTrackingText = config.triggerConfig.entity.TrackingText;
			}

			var trackingTextElement = Utils.getChildByName(this.domObject, "trackingText");
			if (trackingTextElement)
			{
				this.surveyTrackingText = trackingTextElement.value || this.surveyTrackingText;
			}

			return this.surveyTrackingText;
		}

		this.discoverIntroduction = function ()
		{
			if (this.domObject != null)
			{
				var domObject = Utils.getChildById(this.domObject, "DIV_0");

				if (domObject != null)
				{
					this.introduction = new Page(domObject, this);
					this.pages.push(this.introduction);
				}
			}
		}

		this.discoverPages = function ()
		{
			var sections = Utils.getChildNodesByClass(this.domObject, "SURVEYSECTION");

			for (var index = 0; index < sections.length; ++index)
			{
				this.pages.push(new Page(sections[index], this));
			}
		}

		this.discoverThankyou = function ()
		{
			if (this.domObject != null)
			{
				var domObject = Utils.getChildById(this.domObject, "DIV_CLOSE");

				if (domObject != null)
				{
					this.thankyou = new Page(domObject, this);
					this.pages.push(this.thankyou);
				}
			}
		}

		this.discoverIntroduction();
		this.discoverPages();
		this.discoverThankyou();

		this.urlParameters = Utils.getUrlParameters();

		this.addSubmitField = function (key, value)
		{
			this.submitFields[key] = value;
		}

		var currentPageIndex = -1;

		var history = [];
		var variables = [];

		this.populateVariables = function ()
		{
			variables = [];
			for (var index = 0; index < history.length; ++index)
			{
				var page = this.pages[history[index]];
				if (page.modified)
				{
					var subVariables = page.getVariables();
					for (var key in subVariables)
					{
						variables[key] = subVariables[key];
					}
				}
			}
		}

		this.getVariables = function ()
		{
			return variables;
		}

		this.getVariable = function (name)
		{
			return variables[name];
		}

		this.isDefined = function (name)
		{
			if (this.getVariable(name) != null)
			{
				return true;
			}
			else
			{
				return false;
			}
		}

		this.evalute = function (expression)
		{
			var exp = new MS.Support.Fms.SurveyExpression(this, expression);
			return exp.evalute();
		}

		this.EvaluateContextVisibility = function (element)
		{
			var contextCondition = element.getAttribute("showIf");
			if (contextCondition == null || contextCondition == "")
			{
				return "NotExist";
			}
			else
			{
				var retVal = "EvalFalse";
				try
				{
					var expWrapper = new MS.Support.Fms.ExpressionWrapper(contextCondition, this);
					retVal = expWrapper.Execute();
					if (typeof (retVal) == "undefined" || retVal == null)
					{
						retVal = "NotExist";
					}
					else if (retVal.toString() == "true")
					{
						retVal = "EvalTrue";
					}
					else if (retVal.toString() == "false" || !/^(NotExist|EvalTrue|EvalFalse)$/g.test(retVal))
					{
						retVal = "EvalFalse";
					}
				}
				catch (e)
				{
					retVal = "NotExist";
				}
				return retVal;
			}
		}

		this.start = function ()
		{
			if (this.onBeforeStart.fire(this, null))
			{
				this.show();
				if (this.pages.length > 0)
				{
					currentPageIndex = -1;

					this.next();
				}

				if (typeof (this.startTime) == "undefined" || this.startTime == null)
				{
					this.startTime = new Date();
				}

				this.onAfterStart.fire(this, null);
			}
		}

		this.findPageIndexByPageId = function (pageId)
		{
			for (var index = 0; index < this.pages.length; ++index)
			{
				if (this.pages[index].getPageId() == pageId)
				{
					return index;
				}
			}

			return -1;
		}

		this.getPageById = function (pageId)
		{
			var index = this.findPageIndexByPageId(pageId);
			if (index != -1)
			{
				return this.pages[index];
			}

			return null;
		}

		this.getPageByElement = function (element)
		{
			for (var i = 0; i < this.pages.length; ++i)
			{
				if (Utils.isAncestorOf(this.pages[i].domObject, element))
				{
					return this.pages[i];
				}
			}

			return null;
		}

		this.getCurrentPage = function ()
		{
			return this.pages[currentPageIndex];
		}

		this.next = function ()
		{
			if (currentPageIndex >= this.pages.length || (this.thankyou != null && this.pages[currentPageIndex] == this.thankyou))
			{
				return false;
			}

			if (this.onBeforeNext.fire(this, null))
			{
				if (currentPageIndex == -1)
				{
					currentPageIndex = 0;

					if (currentPageIndex >= this.pages.length)
					{
						this.submit();
					}
					else
					{
						var showpage = parseInt(Utils.getChildById(this.domObject, "showpage").value);
						// display the page specified by showpage
						if (showpage > 1 && showpage <= this.pages.length)
						{
							for (var i = 0; i < showpage - 1; i++)
							{
								history.push(i);
							}

							currentPageIndex = showpage - 1;
						}
						// if the value of showpage is out of the pages array, then display the thankyou page
						else if (showpage > this.pages.length)
						{
							currentPageIndex = this.pages.length - 1;
						}

						this.pages[currentPageIndex].show();
						this.pages[currentPageIndex].save();
					}
				}
				else
				{
					var validateResult = this.pages[currentPageIndex].validate();
					if (validateResult.isValid)
					{
						history.push(currentPageIndex);
						this.pages[currentPageIndex].hide();
						this.pages[currentPageIndex].modified = true;
						this.pages[currentPageIndex].populateVariables();
						this.populateVariables();

						var nextPageIndex = -1;

						var nextPageId = this.pages[currentPageIndex].getNextPageByBranchRule();

						if (nextPageId > 0)
						{
							nextPageIndex = this.findPageIndexByPageId(nextPageId);
						}

						if (nextPageIndex != -1 && nextPageIndex != currentPageIndex)
						{
							currentPageIndex = nextPageIndex;
						}
						else
						{
							++currentPageIndex;
						}

						if (currentPageIndex >= this.pages.length || this.thankyou != null && (this.pages[currentPageIndex] == this.thankyou))
						{
							this.submit();
						}
						else
						{
							var page = this.pages[currentPageIndex];
							page.show();
							page.save();
							if (page.ignored)
							{
								this.skip();
							}
						}
					}
					else
					{
						this.onValidateError.fire(this, validateResult);
						return false;
					}
				}

				this.onAfterNext.fire(this, null);
				return true;
			}
			else
			{
				return false;
			}
		}

		this.skip = function ()
		{
			if (this.onBeforeSkip.fire(this, null))
			{
				if (currentPageIndex < 0)
				{
					throw "currentPageIndex < 0";
				}
				else if (this.pages[currentPageIndex].isRequired() && !this.pages[currentPageIndex].ignored)
				{
					throw "Cannot skip a required question.";
				}
				else
				{
					history.push(currentPageIndex);
					this.pages[currentPageIndex].restore();
					this.pages[currentPageIndex].hide();
					this.pages[currentPageIndex].modified = false;
					this.populateVariables();

					var nextPageIndex = -1;

					var nextPageId = this.pages[currentPageIndex].getNextPageByBranchRule();

					if (nextPageId > 0)
					{
						nextPageIndex = this.findPageIndexByPageId(nextPageId);
					}

					if (nextPageIndex != -1 && nextPageIndex != currentPageIndex)
					{
						currentPageIndex = nextPageIndex;
					}
					else
					{
						++currentPageIndex;
					}

					if (currentPageIndex >= this.pages.length || this.thankyou != null && (this.pages[currentPageIndex] == this.thankyou))
					{
						this.submit();
					}
					else
					{
						var page = this.pages[currentPageIndex];
						page.show();
						page.save();
						if (page.ignored)
						{
							this.skip();
						}
					}
				}

				this.onAfterSkip.fire(this, null);
			}
		}

		this.previous = function ()
		{
			if (this.onBeforePrevious.fire(this, null))
			{
				if (currentPageIndex <= 0)
				{
					throw "currentPageIndex <= 0";
				}

				if (history.length == 0)
				{
					throw "history.length == 0";
				}

				var lastPageIndex = history.pop();
				this.pages[currentPageIndex].hide();
				this.pages[currentPageIndex].modified = false;
				currentPageIndex = lastPageIndex;
				this.populateVariables();
				var page = this.pages[currentPageIndex];
				page.show();
				page.save();
				if (page.ignored)
				{
					this.previous();
				}
			}

			this.onAfterPrevious.fire(this, null);
		}

		this.accept = function ()
		{
			if (this.onBeforeAccept.fire(this, null))
			{
				if (this.next())
				{
					this.onAfterAccept.fire();
				}
			}
		}

		this.decline = function ()
		{
			if (this.onBeforeDecline.fire(this, null))
			{
				if (this.cancel())
				{
					this.introduction.hide();
					this.onAfterDecline.fire(this, null);
				}
			}
		}

		this.close = function ()
		{
			if (this.onBeforeClose.fire(this, null))
			{
				window.close();
				this.onAfterClose.fire(this, null);
			}
		}

		this.cancel = function ()
		{
			if (this.onBeforeCancel.fire(this, null))
			{
				this.submit("cancel");
				this.onAfterCancel.fire(this, null);
			}
			else
			{
				return false;
			}

			return true;
		}

		this.giveup = function ()
		{
			if (this.pages[currentPageIndex] != null)
			{
				this.pages[currentPageIndex].restore();
			}
			this.submit("giveup");
		}

		this.stop = function ()
		{
			var parameters = window.location.search.substring(1).split("&");
			var querystring = "";
			for (var i = 0; parameters.length > i; i++)
			{
				if (0 > parameters[i].toLowerCase().indexOf("showtype="))
				{
					querystring += "&" + parameters[i];
				}
			}
			querystring += "&showtype=5";
			window.location.search = "?" + querystring.substring(1);
		}

		this.getSurveyId = function ()
		{
			return this.submitFields["SURVEYID"] || 0;
		}

		this.getSite = function ()
		{
			return this.submitFields["SITE"] || (config ? config.site.name.toUpperCase() : "GN");
		}

		this.getContext = function ()
		{
			return config ? config.context : this.context;
		}

		this.getRegion = function ()
		{
			return this.submitFields["REGIONID"] || "EN-US";
		}

		this.getSurveyLanguageCode = function ()
		{
			return this.submitFields["SURVEYLANGCODE"] || "EN";
		}

		this.getQualifiedFollowUps = function (action)
		{
			var followUpsField = Utils.getChildById(this.domObject, "followups");

			var followUps = FollowUp.parseFollowupsField(followUpsField ? followUpsField.value : "", this.getSurveyLanguageCode(), loadTime);
			var qualifiedFollowUps = [];

			for (var i = 0; i < followUps.length; ++i)
			{
				var followUp = followUps[i];
				if (followUp.evalute(this, !(action == "persist")))
				{
					qualifiedFollowUps.push(followUp);
				}
			}

			return qualifiedFollowUps;
		}

		this.attachFollowUpSurveys = function (action)
		{
			if (this.isInvitation || this.isPreview || this.suppressSubmission || this.isShowAll)
			{
				return;
			}

			var followups = this.getQualifiedFollowUps(action); //get qualified follow-ups of current survey

			if (followups.length > 0)
			{
				var followupCookieKey = "fmsfollowups" + ("ST_" + this.getSite() + "_" + this.getRegion()).toUpperCase();

				// merge unexpired existing follow-ups with new qualified follow-ups
				var existingFollowups = FollowUp.parseFollowupsCookie(Utils.getCookie(followupCookieKey));

				for (var i = 0; i < existingFollowups.length; ++i)
				{
					var followup = existingFollowups[i];
					if (!followup.isExpired())
					{
						followups.push(followup);
					}
				}

				Utils.setCookie(
					followupCookieKey,
				// package survey language, followups(without conditions) and current time ticket into a string
				// e.g., [en,[1488,1489],86400,172800,63357090678,1221465097],[en,[1489], 86400,172800,63357090678,1221465097]]
				// en is the survey language
				// 63357090678 is the server-side time ticket (in seconds, plus the offset from loading to submitting)
				// 122113076175 is the client-side time ticket (in seconds).
					FollowUp.packageFollowupsCookie(followups),
				// cookie expire time, estimate end time of each followup, and return the max value of them
					FollowUp.getMaxFollowupsExpireTime(followups),
					this.cookieDomain
				);
			}
		}

		this.getPreventMultipleResponsesCookieKey = function ()
		{
			return ("fmspmr_" + this.getSurveyId() + "_" + this.getSurveyLanguageCode()).toUpperCase();
		}

		this.attachPreventMultipleResponsesDuration = function (action)
		{
			if (this.isInvitation || this.isPreview || this.suppressSubmission || this.isShowAll || action != "persist")
			{
				return;
			}

			if (this.getSurveyId() == 0)
			{
				return;
			}

			var duration = null;

			var durationField = Utils.getChildById(this.domObject, "noMultipleResponsesDuration");
			if (durationField)
			{
				duration = durationField.value;

				var expires = null;

				if (duration == "session")
				{
					expires = null;
				}
				else if (duration == "permanently")
				{
					expires = new Date();
					expires.setFullYear(expires.getFullYear() + 10);
				}
				else
				{
					var value = parseInt(duration);
					if (isNaN(value))
					{
						return;
					}
					else
					{
						expires = new Date();
						expires.setSeconds(expires.getSeconds() + value);
					}
				}

				Utils.setCookie(this.getPreventMultipleResponsesCookieKey(), "1", expires, this.cookieDomain);
			}
		}

		this.submit = function (action)
		{
			if (typeof (this.startTime) != "undefined" && this.startTime != null && !this.isKBEmbedded && !this.isInvitation)
			{
				var d = new Date();
				this.addSubmitField("SURVEYDWELLTIME", d.getTime() - this.startTime.getTime());
			}

			if (hasSubmitted)
			{
				return;
			}

			if (action == null)
			{
				action = "persist";
			}

			if (this.onBeforeSubmit.fire(this, action))
			{
				this.attachFollowUpSurveys(action);
				this.attachPreventMultipleResponsesDuration(action);
				if (this.submitHandler)
				{
					this.submitHandler(this, action);
				}
				else
				{
					this.defaultSubmitHandler();
				}
				hasSubmitted = true;
				this.onAfterSubmit.fire(this, action);
			}
		}

		this.defaultSubmitHandler = function ()
		{
			// show thankyou page after submited
			if (this.thankyou != null)
			{
				this.thankyou.show();
			}
		}

		this.encodeAnswers = function (textEncoder)
		{
			if (!textEncoder)
			{
				textEncoder = function (input) { return input; };
			}

			var answers = [];

			for (var pageIndex = 0; pageIndex < this.pages.length; ++pageIndex)
			{
				var page = this.pages[pageIndex];
				var pageId = page.getPageId();

				if (pageId != 0 && page.modified == true)
				{
					for (var questionIndex = 0; questionIndex < page.questions.length; ++questionIndex)
					{
						var question = page.questions[questionIndex];
						var questionId = question.getQuestionId();

						var answer = question.getAnswers();

						for (var index = 0; index < answer.length; ++index)
						{
							answers.push(pageId + "," + questionId + "," + answer[index].id + "," + answer[index].value + "," + textEncoder(answer[index].text));
						}
					}
				}
			}

			return answers.join("|");
		}

		this.hide = function ()
		{
			if (this.domObject != null)
			{
				this.domObject.style.display = "none";
			}
		}

		this.show = function ()
		{
			if (this.domObject != null)
			{
				this.domObject.style.display = "block";
			}
		}

		this.displayBranchRules = function ()
		{
			var index = 0;
			for (; index < this.pages.length - 1; ++index)
			{
				this.pages[index].displayBranchRules(this.pages[index + 1].getPageId());
			}

			if (this.thankyou == null)
			{
				this.pages[index].displayBranchRules("End");
			}
		}

		Fms.Survey.RegisterSurveyInstance(this);
	}

	MS.Support.Fms.Survey.SurveyInstances = [];

	MS.Support.Fms.Survey.RegisterSurveyInstance = function (survey)
	{
		MS.Support.Fms.Survey.SurveyInstances[survey.id] = survey;
	}

	MS.Support.Fms.Survey.GetSurveyInstanceById = function (id)
	{
		return MS.Support.Fms.Survey.SurveyInstances[id];
	}

	MS.Support.Fms.Survey.GetSurveyInstanceByElement = function (element)
	{
		var Fms = MS.Support.Fms;
		var Utils = Fms.Utils;
		var Survey = Fms.Survey;

		for (var surveyid in Survey.SurveyInstances)
		{
			var survey = Survey.SurveyInstances[surveyid];
			if (Utils.isAncestorOf(survey.domObject, element))
			{
				return survey;
			}
		}
		return null;
	}
}

function enforceMaxLength(oElement, iMaxLength, e)
{
	if (oElement.value.length > iMaxLength)
	{
		oElement.value = (oElement.value).substring(0, iMaxLength);

		if (window.event) /*Contact Us 2.1 will invoke this function, but only pass the first two parameters,thus e will be undefined. use window.event first*/
		{
			window.event.returnValue = false;
		}
		else if (e && e.preventDefault)
		{
			e.preventDefault();
		}
	}
}

/*backward compatible with KBFeedBack*/
function KBFeedBackShowSurvey()
{
	var gsfxSurveyScript = document.createElement("script");
	gsfxSurveyScript.type = "text/javascript";
	gsfxSurveyScript.language = "javascript";
	gsfxSurveyScript.src = "/common/script/fx/gsfxsurvey.js?4.3.3";
	document.body.appendChild(gsfxSurveyScript);
}
