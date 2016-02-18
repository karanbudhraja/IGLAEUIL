'''
@Description: This code attempts to model the joint learning architecture presented in A Joint Model of Language and Perception for Grounded Attribute Learning
			  Referer to http://cynthia.matuszek.org/Pubs/MatuszekICML2012.pdf, http://cynthia.matuszek.org/Slides/MatuszekFutureOfHRIWorkshopICRA2012.pdf
'''

__author__	= "Karan K. Budhraja,Nisha Pillai"
__email__	= "karanb1@umbc.edu,npillai1@umbc.edu"

# libraries used
from abc import ABCMeta
from abc import abstractmethod
from lib.image.color import detectColor as dc
from lib.image.shape import shapeUtils as su
import sets
from sklearn.linear_model import SGDClassifier
import numpy as np


# ObjWord constants
# possible example polarities
OW_POSITIVE_POLARITY = "+"
OW_NEGATIVE_POLARITY = "-"
# comparison threshold values to consider items as duplicates
# ignoring duplicates right now
OW_COLOR_DUPLICATE_THRESHOLD = 1.1
OW_SHAPE_DUPLICATE_THRESHOLD = 1.1
# comparison threshold values to consider items as matched or mismatched
# this is for classification purposes
OW_COLOR_POSITIVE_EXAMPLE_THRESHOLD = 0.4
OW_COLOR_NEGATIVE_EXAMPLE_THRESHOLD = 0.2
OW_COLOR_CLASSIFICATION_THRESHOLD = 0.9
OW_SHAPE_POSITIVE_EXAMPLE_THRESHOLD = 0.9
OW_SHAPE_NEGATIVE_EXAMPLE_THRESHOLD = 0.2
OW_SHAPE_CLASSIFICATION_THRESHOLD = 0.9

# JointModel constants
JM_GUESS_SCORE_THRESHOLD = 0.8

'''
class created corresponding to classifier for each word
the name is derived from the notation obj-color and obj-shape used in the paper
each object corresponds to a word
each object corresponds to examples related to that word. examples may be positive or negative
'''
class ObjWord:

	# enable creation of abstract methods
	__metaclass__ = ABCMeta

	# creating an object corresponding to a new word
	# word: string
	# example: image
	# example polairty: global definition (constant)
	def __init__(self, word, example, examplePolarity):
		self.word = word
		self.positiveExamples = []
		self.negativeExamples = []
		self.add_example(example, examplePolarity)

	# each derived class must define its item comparison method
	# this is used to compare two items on different features
	# the features are specified by the derived class 

	# compare two items and get a score
	# item 1: image corresponding to word 
	# item 2: image corresponding to word
	@abstractmethod
	def compare_items(self, item1, item2):
		pass

	# score for two duplicate items being compared
	# score varies with comparison method and is therefore class dependent
	@abstractmethod
	def get_duplicate_threshold(self):
		pass

	# score for example compared with positive example
	# score varies with comparison method and is therefore class dependent
	@abstractmethod
	def get_positive_example_threshold(self):
		pass

	# score for example compared with negative example
	# score varies with comparison method and is therefore class dependent
	@abstractmethod
	def get_negative_example_threshold(self):
		pass

	# score for classification with example
	# score varies with comparison method and is therefore class dependent
	@abstractmethod
	def get_classification_threshold(self):
		pass
'''
	# check for duplicates
	# item 1: image corresponding to word
	# item 2: image corresponding to word
	def is_equal(self, item1, item2):
		if(self.compare_items(item1, item2) >= self.get_duplicate_threshold()):
			return True		
		else:
			return False
'''
	# each derived class must define its example comparison method
	# ignore this if duplicate examples are not to be filtered
	# example: candidate example 
	# existing examples: list of known examples
'''	def is_known_example(self, example, existingExamples):
		# initial assumption
		isKnown = False		

		# check for any match
		# more convoluted but faster this way because no if condition
		for existingExample in existingExamples:
			isKnown = isKnown or self.is_equal(example, existingExample)

		# return answer
		return isKnown
'''
	# add a new example
	# example: image 
	# example polairty: global definition (constant)
	def add_example(self, example, examplePolarity, numberOfTimes=1):

		# add multiple times
		for time in range(numberOfTimes):
			if(examplePolarity == OW_POSITIVE_POLARITY):
				# positive example
				if(self.is_known_example(example, self.positiveExamples) == False):
					self.positiveExamples.append(example)
			elif(examplePolarity == OW_NEGATIVE_POLARITY):
				# negative example
				if(self.is_known_example(example, self.negativeExamples) == False):
					self.negativeExamples.append(example)
			else:
				# neither positive nor negative example. ignore
				pass

	# get classification (probability) score for this classifier based on known examples
	def calculate_probability_score(self, example, additionalPositiveExamples=[], additionalNegatveExamples=[]):

		# add additional positive examples if any
		positiveExamples = self.positiveExamples + additionalPositiveExamples
	
		# add additional negative examples if any
		negativeExamples = self.negativeExamples + additionalNegatveExamples
	
		# filter duplicates in examples
		seenExamples = []
		for positiveExample in positiveExamples:
			if(positiveExample not in seenExamples):
				seenExamples.append(positiveExample)
		positiveExamples = seenExamples
			
		seenExamples = []
		for negativeExample in negativeExamples:
			if(negativeExample not in seenExamples):
				seenExamples.append(negativeExample)
		negativeExamples = seenExamples
		
		# check against this classifier
		correctExamples = 0

		# check against positive examples
		for positiveExample in positiveExamples:
			#print("in positive example" + str(self.compare_items(example, positiveExample)) + str(self.get_positive_example_threshold()))
			if(self.compare_items(example, positiveExample) > self.get_positive_example_threshold()):
				correctExamples += 1

		# check against negative examples
		for negativeExample in negativeExamples:
			#print("in negative example")
			#print(self.compare_items(example, negativeExample))
			if(self.compare_items(example, negativeExample) < self.get_negative_example_threshold()):
				correctExamples += 1

		# compute p(example|word)
		#print("correct examples: " + str(correctExamples))
		totalExamples = len(positiveExamples) + len(negativeExamples)
		pExampleGivenWord = correctExamples/float(totalExamples)

		# p(word) = totalExamples / examples over all worlds
		# the denominator is constant for all word scores. ignore it
		# consider non-normalized version of p(word) to calculate score
		# probabilityScore = pExampleGivenWord*totalExamples cancels totalExamples
		# so we can use just correctExamples
		probabilityScore = correctExamples

		# using pExampleGivenWord for now
		#probabilityScore = pExampleGivenWord

		# return the score
		return [probabilityScore, pExampleGivenWord]

'''
initialization: each word may correspond to a color, shape or synonym
null hypothesis is not a class. it is a conclusion if no class matches
'''
class ObjColor(ObjWord):
	
	# static type definition
	_type_ = "Color"

	# static computation table
	_computation_table_ = {}

	# compare two items in terms of colors and get a score
	# item 1: image 
	# item 2: image 
	def compare_items(self, item1, item2):

		# check if the value is already computed
		combinationName1 = str(item1) + str(item2)
		combinationName2 = str(item2) + str(item1)
	
		if(combinationName1 in ObjColor._computation_table_.keys()):
			return ObjColor._computation_table_[combinationName1]
		elif(combinationName2 in ObjColor._computation_table_.keys()):
			return ObjColor._computation_table_[combinationName2]
		else:
			# exctract colors from two images and compare them		
			ObjColor._computation_table_[combinationName1] = dc.compare_items(item1['color'], item2['color'])
			return ObjColor._computation_table_[combinationName1]

	# score for two duplicate items being compared
	# score varies with comparison method and is therefore class dependent
	def get_duplicate_threshold(self):
		return OW_COLOR_DUPLICATE_THRESHOLD

	# score for example compared with positive example
	# score varies with comparison method and is therefore class dependent
	def get_positive_example_threshold(self):
		return OW_COLOR_POSITIVE_EXAMPLE_THRESHOLD

	# score for example compared with negative example
	# score varies with comparison method and is therefore class dependent
	def get_negative_example_threshold(self):
		return OW_COLOR_NEGATIVE_EXAMPLE_THRESHOLD

	# score for classification with example
	# score varies with comparison method and is therefore class dependent
	def get_classification_threshold(self):
		return OW_COLOR_CLASSIFICATION_THRESHOLD

class ObjShape(ObjWord):

	# static type definition
	_type_ = "Shape"
	
	# static computation table
	_computation_table_ = {}

	# compare two items in terms of shape and get a score
	# item 1: image 
	# item 2: image
	def compare_items(self, item1, item2):
	
		# check if the value is already computed
		combinationName1 = str(item1) + str(item2)
		combinationName2 = str(item2) + str(item1)
	
		if(combinationName1 in ObjShape._computation_table_.keys()):
			return ObjShape._computation_table_[combinationName1]
		elif(combinationName2 in ObjShape._computation_table_.keys()):
			return ObjShape._computation_table_[combinationName2]
		else:
			# exctract shapes from two images and compare them		
			ObjShape._computation_table_[combinationName1] = su.compare_items(item1['shape'], item2['shape'])
			return ObjShape._computation_table_[combinationName1]

	# score for two duplicate items being compared
	# score varies with comparison method and is therefore class dependent
	def get_duplicate_threshold(self):
		return OW_SHAPE_DUPLICATE_THRESHOLD

	# score for example compared with positive example
	# score varies with comparison method and is therefore class dependent
	def get_positive_example_threshold(self):
		return OW_SHAPE_POSITIVE_EXAMPLE_THRESHOLD

	# score for example compared with negative example
	# score varies with comparison method and is therefore class dependent
	def get_negative_example_threshold(self):
		return OW_SHAPE_NEGATIVE_EXAMPLE_THRESHOLD

	# score for classification with example
	# score varies with comparison method and is therefore class dependent
	def get_classification_threshold(self):
		return OW_SHAPE_CLASSIFICATION_THRESHOLD

class ObjSynonymColor(ObjWord):

	# re-uses some of ObjWord but not as much as Color and Shape do
	# only re-uses example addition code

	# static type definition
	_type_ = "Synonym Color"

	# creating a word as a synonym for another word
	# word: string
	# synonym: string
	# we do not use the other constructor
	# try to use @override
	def __init__(self, word, synonym, example, examplePolarity):
		self.word = word
		self.synonym = synonym
		self.positiveExamples = []
		self.negativeExamples = []
		self.add_example(example, examplePolarity)

	# dummy implementations of abstract methods
	# needed to instantiate ObjSynonym
	# not used by ObjSynonym objects
	def compare_items(self, item1, item2):
		pass
	def get_duplicate_threshold(self):
		pass
	def get_positive_example_threshold(self):
		pass
	def get_negative_example_threshold(self):
		pass
	def get_classification_threshold(self):
		pass

class ObjSynonymShape(ObjWord):

	# re-uses some of ObjWord but not as much as Color and Shape do
	# only re-uses example addition code

	# static type definition
	_type_ = "Synonym Shape"

	# creating a word as a synonym for another word
	# word: string
	# synonym: string
	# we do not use the other constructor
	# try to use @override
	def __init__(self, word, synonym, example, examplePolarity):
		self.word = word
		self.synonym = synonym
		self.positiveExamples = []
		self.negativeExamples = []
		self.add_example(example, examplePolarity)

	# dummy implementations of abstract methods
	# needed to instantiate ObjSynonym
	# not used by ObjSynonym objects
	def compare_items(self, item1, item2):
		pass
	def get_duplicate_threshold(self):
		pass
	def get_positive_example_threshold(self):
		pass
	def get_negative_example_threshold(self):
		pass
	def get_classification_threshold(self):
		pass

'''
joint model class which will contain many word based classifiers
for synonyms, two classifiers may have large confidence in a situation. merging of examples should then be considered
we will not deal with synonyms for now
since the model is data driven, we do not need to train classifiers. we simply store data an calculate at test time
'''
class JointModel:

	# creating an empty model
	def __init__(self):
	
		# known words and their classifiers
		self.knownWords = {}
		self.minimumGuessScore = JM_GUESS_SCORE_THRESHOLD
                self.clfColor = SGDClassifier(loss="log", penalty="l2")
                self.clfShape = SGDClassifier(loss="log", penalty="l2")
                self.classColors = []
                self.classShapes = []

        # add a word-example pair to the SGD classifer
        # word: string
        # example: image
        def add_sgd_class(self, word, example):
           self.clfColor = SGDClassifier(loss="log", penalty="l2")
           self.clfShape = SGDClassifier(loss="log", penalty="l2")
           X_Color = [example['Color']]
           y_Color = [word]
           X_Shape = [example['Shape']]
           y_Shape = [word]
           for word in self.knownWords.keys():
              for classifier in self.knownWords[word]:
                 if("Synonym" not in str(type(classifier))):
                    examples = classifier.positiveExamples
                    for ex in examples : 
                       if("Color" in classifier._type_):
                          X_Color.append(ex['Color'])
                          y_Color.append(word)
                       if("Shape" in classifier._type_):
                          X_Shape.append(ex['Shape'])
                          y_Shape.append(word)
           
           classes = np.unique(y_Color)
           self.clfColor.partial_fit(X_Color, y_Color,classes=classes)
           self.classColors = classes
           classes = np.unique(y_Shape)
           self.clfShape.partial_fit(X_Shape, y_Shape,classes=classes)
           self.classShapes = classes

        # add a word-example pair to existing SGD classifer
        # word: string
        # example: image
        def partial_fit_classifer(self,word,example):
           self.clfColor.partial_fit([example['Color']],[word])
           self.clfShape.partial_fit([example['Shape']],[word])



	# add a word-example pair to the model
	# word: string
	# example: image
	# example polairty: global definition (constant)
	def add_word_example_pair(self, word, example, examplePolarity):
	
		currentKnownWords = self.knownWords.keys()
                
                # check if it is a new word
		if(word not in self.knownWords.keys()):
			# new word. add possibly associated classifiers
			# limited to initialization
			self.knownWords[word] = []
			self.knownWords[word].append(ObjColor(word, example, examplePolarity))
			self.knownWords[word].append(ObjShape(word, example, examplePolarity))

                        self.add_sgd_class(word,example)
                                         
	                # add possibilities of being a synonym
			# this will not contain redundant information like (a b), (a c) and (b c)
			# this is because syonyms are added in order
			for knownWord in currentKnownWords:
				# word may be a synonym of knownWord
				# when classifying, synonyms are checked for all classifier types
				# e.g. color, shape
				self.knownWords[word].append(ObjSynonymColor(word, knownWord, example, examplePolarity))
				self.knownWords[word].append(ObjSynonymShape(word, knownWord, example, examplePolarity))
		else:
                        self.partial_fit_classifer(word,example)
			# known word. just add the example
			# add in all word objects (where adding an example is possible)			
			for classifier in self.knownWords[word]:
				# assume all types to qualify for example addition
				classifier.add_example(example, examplePolarity)

	'''
	experiment: trained attributes
	'''
	# classify a word with corresponding example and get positive or negative confirmation
	# if the classifier is confident, then we don't know
	# e.g. "is this green?"
	# word: string
	# example: image
	# classificationScores: dictionary of classification scores per classifier
	def classify_word_example(self, word, example, checkSynonyms=True):
                probColor = 0.0
                probShape = 0.0
                if(word in self.classColors) :
                   index = self.classColors(word)
                   colorPredict = self.clfColor.predict([example['Color']])
                   colorProbs   = self.clfColor.predict_proba([example['Color']])
                   probColor = colorProbs[index]

                if(word in self.classShapes) :
                   index = self.classShapes(word)
                   shapePredict = self.clfShape.predict([example['Shape']])
                   shapeProbs   = self.clfShape.predict_proba([example['Shape']])
                   probShape = shapeProbs[index]
               

		probabilityScores = {}
		pExampleGivenWordValues = {}

		# check all classifiers related to this word
		for classifier in self.knownWords[word]:
			#print(word, str(classifier))
			if("Synonym" not in str(type(classifier))):
				# use non-synonym classifiers directly
				[probabilityScore, pExampleGivenWord] = classifier.calculate_probability_score(example)
			elif(checkSynonyms == True):
				# use synonym classifiers indirectly
				# add positive and negative examples known for the word but not the synonym
				# we do not care about the return values for recursive calls
				# we only want to populate probabilityScores in each recursion
	
				if("Color" in str(type(classifier))):
					searchType = "ObjColor"					
				elif("Shape" in str(type(classifier))):
					searchType = "ObjShape"					
				else:
					# should never come here for given initialization
					pass

				for synonymClassifier in self.knownWords[classifier.synonym]:
					if(searchType in str(type(synonymClassifier))):
						# will only enter this once
						# break is efficient but not neccessary
						synonymClassifierObj = synonymClassifier
						break;

				[probabilityScore, pExampleGivenWord] = synonymClassifierObj.calculate_probability_score(example, classifier.positiveExamples, classifier.negativeExamples)
			
			else:
				# do nothing about the synonyms
				pass

			# add score to classification scores
			probabilityScores[classifier] = probabilityScore
			pExampleGivenWordValues[classifier] = pExampleGivenWord

		# now we have accumulated all the scores
		# check if any of the scores exceed the threshold
		# initially assume inconsistency
		isWordExampleConsistent = False

		# compare for positive
		# more convoluted but faster this way because no if condition
		for classifier in probabilityScores.keys():
			isWordExampleConsistent = isWordExampleConsistent or (probabilityScores[classifier] >= classifier.get_classification_threshold())

		# return the consistency decision and probability scores
		return [isWordExampleConsistent, probabilityScores, pExampleGivenWordValues]

	'''
	experiment: novel scene
	'''	
	# classify a new example and get corresponding word
	# if no classifier is confident, then it is a new category of example. we do not handle this right now
	# e.g. "what is this?"
	# e.g. of bayes' rule: p(cube|example) = p(example|cube) * p(cube) / p(example)
	# p(example) is constant across all word classifications and can be ignored when comparing them
	# p(example|cube): the fraction of examples in "cube" which matched the current example
	# p(cube): the fraction of examples under "cube" relative to examples over all known words
	# p(cube) = totalExamples of cube / total examples of all words
	# the denominator is constant for all word scores. ignore it
	# consider non-normalized version of p(cube) to calculate score
	# example: image
	def classify_example(self, example):


                colorPredict = self.clfColor.predict([example['Color']])
                colorProbs   = self.clfColor.predict_proba([example['Color']])

                shapePredict = self.clfShape.predict([example['Shape']])
                shapeProbs   = self.clfShape.predict_proba([example['Shape']])
		# check against each known word
		# maximum probability score data corresponding to a word		
		wordMaxProabilityScores = {}
		# all probability score data corresponding to a word
		wordProbabilityScores = {}

		# maintain best guess
		bestGuessWord = ""
		bestGuessObj = ""
		bestGuessMaxScore = 0
	
		# calculate word probability scores
		# check all associated classifiers
		for word in self.knownWords.keys():
			[isWordExampleConsistent, probabilityScores, pExampleGivenWordValues] = self.classify_word_example(word, example)

			# select maximum score corresponding to best interpretation			
			maxScore = max(probabilityScores.values())			

			maxScoreObj = "none"
			for classifier in probabilityScores:
				if(probabilityScores[classifier] == maxScore):
					maxScoreObj = classifier

			# add to probability scores
			#totalObjExamples = float(len(maxScoreObj.positiveExamples) + len(maxScoreObj.negativeExamples))
			#wordMaxProabilityScores[word] = [maxScore, maxScoreObj, maxScore/totalObjExamples]
			wordMaxProabilityScores[word] = [maxScore, maxScoreObj, pExampleGivenWordValues]
			wordProbabilityScores[word] = [isWordExampleConsistent, probabilityScores]

			# update best guess if possible
			if(maxScore > bestGuessMaxScore):
				bestGuessWord = word
				bestGuessObj = maxScoreObj
				bestGuessMaxScore = maxScore

		# guess confidence
		# initial assumption
		isConfidentGuess = False

		if(bestGuessMaxScore >= self.minimumGuessScore):
			isConfidentGuess = True
		
		# return everything known to man
		return [isConfidentGuess, bestGuessWord, bestGuessObj, bestGuessMaxScore, wordMaxProabilityScores, wordProbabilityScores]
		
	'''
	experiment: novel english
	'''	
	# get a sentence and an image
	# compute a score which represents association of words with that image
	# classify that image and get words associated with it in decending order
	# get ranks of word mentioned by user
	# score is sum of 1/rank for each word
	# e.g. "this is a blue cube"
	def associate_words_example(self, listOfPositiveWords, listOfNegativeWords, example):
		
		# classify this image and get associated words
		[isConfidentGuess, bestGuessWord, bestGuessObj, bestGuessMaxScore, wordMaxProabilityScores, wordProbabilityScores] = self.classify_example(example)

		# form a dictionary of score: word
		wordScoreDictionary = {}
		for word in wordMaxProabilityScores:
			if(wordMaxProabilityScores[word][2][wordMaxProabilityScores[word][1]] not in wordScoreDictionary.keys()):
			    wordScoreDictionary[wordMaxProabilityScores[word][2][wordMaxProabilityScores[word][1]]] = [word]
			else:
			    wordScoreDictionary[wordMaxProabilityScores[word][2][wordMaxProabilityScores[word][1]]].append(word)

		# now rank these in descending order
		rank = 0
		wordRanks = {}
		for wordScore in sorted(wordScoreDictionary.keys(),reverse=True):
			rank += 1
			for word in wordScoreDictionary[wordScore]:
			    wordRanks[word] = rank

		# compute total score based on ranks of words in list
		totalScore = 0
		for word in listOfPositiveWords:
			if(word in wordRanks):
				rank = wordRanks[word]
				# use flat division for float result
				totalScore += 1.0/rank

		for word in listOfNegativeWords:
			if(word in wordRanks):
				rank = wordRanks[word]
				# use flat division for float result
				totalScore -= 1.0/rank

		return [totalScore, wordRanks, wordScoreDictionary]
